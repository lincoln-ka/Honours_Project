-- data_logger.lua
-- When start_transition (set by vtol_transition_trigger.lua) fires, begins
-- logging two global arrays at 10 Hz until start_transition is cleared.
--
-- state_log[t]      = { lat_dist, lat_vel, height, height_rate,
--                        pitch, pitch_rate, m1, m2, m3, m4, elev_rad }
--
-- input_rate_log[t] = { dm1_dt, dm2_dt, dm3_dt, dm4_dt, delev_dt }
--
-- Indices match the row ordering of Y_test.csv / U_test.csv.

-- ── Configuration ────────────────────────────────────────────────────────────

local LOG_HZ      = 100          -- logging rate
local LOG_MS      = 1000 / LOG_HZ

-- ArduPilot SRV_Channel function numbers — adjust to match your airframe.
local FN_MOTOR1   = 33          -- SRV_Channel::k_motor1
local FN_MOTOR2   = 34          -- k_motor2
local FN_MOTOR3   = 35          -- k_motor3
local FN_MOTOR4   = 36          -- k_motor4
local FN_ELEVATOR = 19          -- k_elevator  (use 41 for tilt-rotor tilt servo)

-- Elevator PWM->radians: PWM 1500 = 0 rad, ±500 PWM = ±MAX_ELEV_RAD.
local MAX_ELEV_RAD = 0.4363     -- ~25 degrees; edit to match your servo throw

-- ── Global log arrays (readable from other scripts) ──────────────────────────

state_log      = {}
input_rate_log = {}

-- ── Module state ─────────────────────────────────────────────────────────────

local _logging   = false
local _last_ms   = 0
local _vtol_loc  = nil   -- cached VTOL_LAND location, set on first active tick

local _prev_m1, _prev_m2, _prev_m3, _prev_m4, _prev_elev = 0, 0, 0, 0, 0
local _prev_lat_dist = nil

-- ── Helpers ───────────────────────────────────────────────────────────────────

local function pwm_to_throttle_pct(pwm)
    if not pwm or pwm < 900 then return 0 end
    return math.max(0, math.min(100, (pwm - 1000) / 10))
end

local function pwm_to_elev_rad(pwm)
    if not pwm or pwm < 900 then return 0 end
    return (pwm - 1500) / 500 * MAX_ELEV_RAD
end

-- Reads the current nav item; returns the Location of VTOL_LAND or nil.
local function get_vtol_land_loc()
    local idx = mission:get_current_nav_index()
    if not idx then return nil end
    local item = mission:get_item(idx)
    if not item or item:command() ~= 85 then return nil end
    local base = ahrs:get_location()
    if not base then return nil end
    base:lat(item:x())
    base:lng(item:y())
    base:alt(math.floor(item:z() * 100))  -- metres -> cm
    return base
end

-- ── Main update loop ──────────────────────────────────────────────────────────

local function update()

    -- ── Flag cleared: stop logging ──────────────────────────────────────────
    if not start_transition then
        if _logging then
            _logging  = false
            _vtol_loc = nil
            _prev_lat_dist = nil
            gcs:send_text(6, string.format(
                "data_logger: stopped — %d samples in state_log, %d in input_rate_log",
                #state_log, #input_rate_log
            ))
        end
        return update, LOG_MS
    end

    -- ── First tick after flag set: initialise ───────────────────────────────
    if not _logging then
        _vtol_loc = get_vtol_land_loc()
        if not _vtol_loc then
            gcs:send_text(3, "data_logger: start_transition set but no VTOL_LAND nav command found")
            return update, LOG_MS
        end
        state_log      = {}
        input_rate_log = {}
        _prev_lat_dist = nil
        _prev_m1, _prev_m2, _prev_m3, _prev_m4, _prev_elev = 0, 0, 0, 0, 0
        _last_ms = millis()
        _logging = true
        gcs:send_text(6, string.format(
            "data_logger: logging started at %d Hz", LOG_HZ
        ))
        return update, LOG_MS
    end

    -- ── Rate gate: honour LOG_HZ ────────────────────────────────────────────
    local now_ms = millis()
    if now_ms - _last_ms < LOG_MS then
        return update, 5   -- spin fast so we don't overshoot the gate
    end
    local dt = math.max((now_ms - _last_ms) / 1000, 1e-6)  -- actual seconds
    _last_ms = now_ms

    -- ── Sensors ─────────────────────────────────────────────────────────────

    local current_loc = ahrs:get_location()
    if not current_loc then
        return update, 5
    end

    -- Lateral distance: 2-D horizontal distance to VTOL_LAND waypoint (m).
    -- Negative lateral_vel = closing on the waypoint.
    local lat_dist = current_loc:get_distance(_vtol_loc)
    local lat_vel  = _prev_lat_dist and (lat_dist - _prev_lat_dist) / dt or 0
    _prev_lat_dist = lat_dist

    -- Height above ground (m): prefer terrain module, fall back to baro
    -- relative to the VTOL_LAND waypoint altitude.
    local height = terrain:height_above_terrain(false)
    if not height then
        local curr_alt_m = current_loc:alt() / 100
        local land_alt_m = _vtol_loc:alt() / 100
        height = curr_alt_m - land_alt_m
    end

    -- Vertical velocity: NED frame, so height rate = -vel_down.
    local height_rate = 0
    local vel_ned = ahrs:get_velocity_NED()
    if vel_ned then
        height_rate = -vel_ned:z()
    end

    -- Attitude.
    local pitch      = ahrs:get_pitch()           -- rad
    local gyro       = ahrs:get_gyro()
    local pitch_rate = gyro and gyro:y() or 0     -- rad/s, body-frame y

    -- Motor throttle outputs (%).
    local m1 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR1))
    local m2 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR2))
    local m3 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR3))
    local m4 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR4))

    -- Elevator / tilt angle (rad).
    local elev = pwm_to_elev_rad(SRV_Channels:get_output_pwm(FN_ELEVATOR))

    -- ── Rates of change (control input derivatives) ──────────────────────────

    local dm1_dt   = (m1   - _prev_m1)   / dt
    local dm2_dt   = (m2   - _prev_m2)   / dt
    local dm3_dt   = (m3   - _prev_m3)   / dt
    local dm4_dt   = (m4   - _prev_m4)   / dt
    local delev_dt = (elev - _prev_elev) / dt

    _prev_m1, _prev_m2, _prev_m3, _prev_m4, _prev_elev = m1, m2, m3, m4, elev

    -- ── Append to log arrays ────────────────────────────────────────────────

    local t = #state_log + 1

    state_log[t] = {
        lat_dist,   -- [1]  lateral distance to VTOL_LAND WP (m)
        lat_vel,    -- [2]  lateral velocity, -ve = closing (m/s)
        height,     -- [3]  height above ground (m)
        height_rate,-- [4]  height rate, +ve = climbing (m/s)
        pitch,      -- [5]  pitch angle (rad)
        pitch_rate, -- [6]  pitch rate, body-y axis (rad/s)
        m1,         -- [7]  motor 1 throttle (%)
        m2,         -- [8]  motor 2 throttle (%)
        m3,         -- [9]  motor 3 throttle (%)
        m4,         -- [10] motor 4 throttle (%)
        elev,       -- [11] elevator / tilt angle (rad)
    }

    input_rate_log[t] = {
        dm1_dt,     -- [1]  motor 1 throttle rate (%/s)
        dm2_dt,     -- [2]  motor 2 throttle rate (%/s)
        dm3_dt,     -- [3]  motor 3 throttle rate (%/s)
        dm4_dt,     -- [4]  motor 4 throttle rate (%/s)
        delev_dt,   -- [5]  elevator / tilt rate (rad/s)
    }

    -- Status every 5 s.
    if t % (LOG_HZ * 5) == 0 then
        gcs:send_text(7, string.format(
            "data_logger: %d samples | dist=%.1fm h=%.1fm pitch=%.3f elev=%.3f",
            t, lat_dist, height, pitch, elev
        ))
    end

    return update, 5   -- spin fast; rate gate above enforces 10 Hz
end

gcs:send_text(6, "data_logger: script loaded")
return update()
