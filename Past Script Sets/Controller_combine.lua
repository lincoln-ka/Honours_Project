-- Controller_combine.lua
-- Combined: csv_loader + vtol_transition_trigger + data_logger_tilt + Controller_vinf

-- ═══════════════════════════════════════════════════════════
-- CONFIGURATION
-- ═══════════════════════════════════════════════════════════

-- CSV loader
local CSV_DIR = "scripts/"

-- Transition trigger
local VTOL_LAND_CMD     = 85
local TRANSITION_DIST_M = 300
local LOG_START_DIST_M  = TRANSITION_DIST_M + 20
local AUTO_MODE         = 10

-- Data logger
local LOG_HZ      = 100
local LOG_MS      = 1000 / LOG_HZ

local FN_MOTOR1   = 33
local FN_MOTOR2   = 34
local FN_MOTOR3   = 35
local FN_MOTOR4   = 36
local FN_TILT     = 41
local FN_ELEVATOR = 19

local MAX_TILT_RAD = 1.5708
local MAX_ELEV_RAD = 0.4363

-- Controller
local sim_time_total   = 17.7681928636437
local sim_time_steps   = 33
local frame_type       = "tilt"
    -- "tilt"   : quad-tilt
    -- "pusher" : quad + pusher/puller
local gain_td          = 0.1
local sim_td           = sim_time_total / sim_time_steps
local log_td           = 10

local m1_chl = 7
local m2_chl = 5
local m3_chl = 6
local m4_chl = 8
local tilt_chl = 9
local tilt_chl_2= 10
local elv_chl  = 2

local quad_rpm_min     = 0
local quad_rpm_max     = 10000
local pusher_rpm_min   = 0
local pusher_rpm_max   = 10000
local k_roll           = 20 * 180 / math.pi
local expo             = 0.5
local elevator_abs_max = math.pi / 4
local tilt_max         = math.pi / 2

local update_interval_motors   = 3.3   -- ~300 Hz
local update_interval_elevator = 100   -- 10 Hz
local update_interval_tilt     = 100   -- 10 Hz

local UPDATE_RATE_MS = 5   -- fast spin; each section gates its own rate

-- ═══════════════════════════════════════════════════════════
-- STATE
-- ═══════════════════════════════════════════════════════════

-- CSV data
local K, U_d, Y_d

-- Transition trigger
local start_transition   = false
local _trig_triggered    = false
local _log_triggered     = false
local _trig_last_dbg_ms  = 0

-- Data logger
local state_log          = {}
local input_rate_log     = {}
local _log_active        = false
local _log_last_ms       = 0
local _log_vtol_loc      = nil
local _log_prev_lat_dist = nil
local _log_prev_m1, _log_prev_m2, _log_prev_m3, _log_prev_m4 = 0, 0, 0, 0
local _log_prev_tilt, _log_prev_elev = 0, 0

-- Debug rate-limiter (shared across sections)
local _dbg_last_ms = 0

-- Controller
local motor_overide_flag    = false
local elevator_overide_flag = false
local tilt_overide_flag     = false
local u_cmd_1_pwm, u_cmd_2_pwm, u_cmd_3_pwm, u_cmd_4_pwm, u_cmd_5_pwm
local u1_prev, u2_prev, u3_prev, u4_prev, u5_prev
local u_elevator_prev, u_tilt_prev
local t_0 = 0
local z_flag = 0
local now = 0
local update_timestamp_motors   = 0
local update_timestamp_elevator = 0
local update_timestamp_tilt     = 0

-- ═══════════════════════════════════════════════════════════
-- HELPERS: CSV LOADER
-- ═══════════════════════════════════════════════════════════

local function parse_csv(filepath)
    local file = io.open(filepath, "r")
    if not file then
        gcs:send_text(3, "combine: cannot open " .. filepath)
        return nil
    end
    local data = {}
    for line in file:lines() do
        if line ~= "" then
            local row = {}
            for field in line:gmatch("[^,]+") do
                local num = tonumber(field)
                if num then row[#row + 1] = num end
            end
            if #row > 0 then data[#data + 1] = row end
        end
    end
    file:close()
    return data
end

-- ═══════════════════════════════════════════════════════════
-- HELPERS: DATA LOGGER
-- ═══════════════════════════════════════════════════════════

local function pwm_to_throttle_pct(pwm)
    if not pwm or pwm < 900 then return 0 end
    return math.max(0, math.min(100, (pwm - 1000) / 10))
end

local function pwm_to_rad(pwm, max_rad)
    if not pwm or pwm < 900 then return 0 end
    return (pwm - 1500) / 500 * max_rad
end

local function get_vtol_land_loc()
    local idx = mission:get_current_nav_index()
    if not idx then return nil end
    local item = mission:get_item(idx)
    if not item or item:command() ~= 85 then return nil end
    local base = ahrs:get_location()
    if not base then return nil end
    base:lat(item:x())
    base:lng(item:y())
    base:alt(math.floor(item:z() * 100))
    return base
end

-- ═══════════════════════════════════════════════════════════
-- HELPERS: CONTROLLER
-- ═══════════════════════════════════════════════════════════

local function init_c()
    if z_flag ~= 1 then
        t_0    = millis()
        z_flag = 1
        gcs:send_text(1, "T_0 set!_actual")
    end
    gcs:send_text(1, "T_0 set!")
end

local function mat_sub(A, B)
    local C = {}
    for i = 1, #A do
        C[i] = {}
        for j = 1, #A[i] do
            C[i][j] = A[i][j] - B[i][j]
        end
    end
    return C
end

local function mat_mul(A, B)
    local m, k, n = #A, #B, #B[1]
    local C = {}
    for i = 1, m do
        C[i] = {}
        for j = 1, n do
            local s = 0
            for p = 1, k do s = s + A[i][p] * B[p][j] end
            C[i][j] = s
        end
    end
    return C
end

local function interpolate(func, now_p, dataset)
    local x1, x3
    if dataset == "gain" then
        x1 = math.floor(now_p / gain_td)
        x3 = now_p / gain_td
    elseif dataset == "sim" then
        x1 = math.floor(now_p / sim_td)
        x3 = now_p / sim_td
    elseif dataset == "log" then
        x1 = math.floor(now_p / log_td)
        x3 = now_p / log_td
    end
    local x2 = x1 + 1
    local w1 = x3 - x1
    local w2 = x2 - x3
    return w2 * func[x1] + w1 * func[x2]
end

local function Expo(thr_pct)
    local thr_norm = thr_pct / 100
    local thr_expo = thr_norm^3 * expo + thr_norm * (1 - expo)
    return 1000 + thr_expo * 1000
end

local function rpm_to_pct(rpm, min, max)
    return (rpm - min) / (max - min)
end

local function pct_to_rpm(thr_pct, min_rpm, max_rpm)
    local norm_e = thr_pct / 100
    local norm   = 0.6826 * (norm_e^3) - 1.7606 * (norm_e^2) + 2.0796 * norm_e + 0.0011
    return norm * (max_rpm - min_rpm)
end

-- ═══════════════════════════════════════════════════════════
-- STARTUP: LOAD CSV FILES
-- ═══════════════════════════════════════════════════════════

gcs:send_text(6, "Controller_combine: loading CSV files...")

local function load_csv(filename, label)
    local path   = CSV_DIR .. filename .. ".csv"
    local result = parse_csv(path)
    if result then
        gcs:send_text(6, string.format(
            "combine: %s loaded (%d rows x %d cols)", label, #result, #result[1]
        ))
    else
        gcs:send_text(3, "combine: " .. label .. " failed to load")
    end
    return result
end

K   = load_csv("K", "K")
U_d = load_csv("U",      "U_d")
Y_d = load_csv("Y",      "Y_d")

gcs:send_text(6, "Controller_combine: CSV files loaded")

-- ═══════════════════════════════════════════════════════════
-- COMBINED UPDATE LOOP
-- ═══════════════════════════════════════════════════════════

local function update()

    local now_ms = millis()

    -- ── Rate-limited global status (1 Hz) ──────────────────────────────────
    if now_ms - _dbg_last_ms >= 1000 then
        _dbg_last_ms = now_ms
        gcs:send_text(6, string.format(
            "[CC] mode=%d start_tran=%s log_trig=%s z_flag=%d csv=%s/%s/%s",
            vehicle:get_mode(),
            tostring(start_transition), tostring(_log_triggered), z_flag,
            tostring(K ~= nil), tostring(U_d ~= nil), tostring(Y_d ~= nil)
        ))
    end

    -- ── 1. Transition trigger ───────────────────────────────────────────────
    if vehicle:get_mode() ~= AUTO_MODE then
        if _trig_triggered then
            _trig_triggered  = false
            _log_triggered   = false
            start_transition = false
            gcs:send_text(6, "[CC] trigger: left AUTO mode — all flags reset")
        end
    else
        local current_idx = mission:get_current_nav_index()
        local nav_item    = current_idx and mission:get_item(current_idx)

        if nav_item and nav_item:command() == VTOL_LAND_CMD then
            if not _trig_triggered then
                local current_loc = ahrs:get_location()
                local vtol_loc    = ahrs:get_location()
                if not current_loc or not vtol_loc then
                    gcs:send_text(4, "[CC] trigger: could not get vehicle location")
                else
                    vtol_loc:lat(nav_item:x())
                    vtol_loc:lng(nav_item:y())
                    local dist_m = current_loc:get_distance(vtol_loc)
                    if now_ms - _trig_last_dbg_ms >= 1000 then
                        _trig_last_dbg_ms = now_ms
                        gcs:send_text(7, string.format(
                            "[CC] VTOL_LAND WP %d: dist=%.1f m  log@%dm  tran@%dm",
                            current_idx, dist_m, LOG_START_DIST_M, TRANSITION_DIST_M
                        ))
                    end
                    if not _log_triggered and dist_m <= LOG_START_DIST_M then
                        _log_triggered = true
                        gcs:send_text(6, string.format(
                            "[CC] trigger: logger armed at %.1f m (WP %d)",
                            dist_m, current_idx
                        ))
                    end
                    if dist_m <= TRANSITION_DIST_M then
                        start_transition = true
                        _trig_triggered  = true
                        gcs:send_text(6, string.format(
                            "[CC] trigger: start_transition SET at %.1f m (WP %d)",
                            dist_m, current_idx
                        ))
                    end
                end
            end
        else
            if _trig_triggered then
                _trig_triggered  = false
                _log_triggered   = false
                start_transition = false
                gcs:send_text(6, "[CC] trigger: nav target no longer VTOL_LAND — flags reset")
            end
        end
    end

    -- ── 2. Data logger ──────────────────────────────────────────────────────
    if not _log_triggered then
        if _log_active then
            _log_active        = false
            _log_vtol_loc      = nil
            _log_prev_lat_dist = nil
            gcs:send_text(6, string.format(
                "[CC] logger: stopped — %d state samples, %d input samples",
                #state_log, #input_rate_log
            ))
        end
    else
        -- First tick: initialise
        if not _log_active then
            gcs:send_text(6, "[CC] logger: initialising...")
            _log_vtol_loc = get_vtol_land_loc()
            if not _log_vtol_loc then
                gcs:send_text(3, "[CC] logger: no VTOL_LAND waypoint found — cannot start")
            else
                state_log          = {}
                input_rate_log     = {}
                _log_prev_lat_dist = nil
                _log_prev_m1, _log_prev_m2, _log_prev_m3, _log_prev_m4 = 0, 0, 0, 0
                _log_prev_tilt, _log_prev_elev = 0, 0
                _log_last_ms = now_ms
                _log_active  = true
                gcs:send_text(6, string.format("[CC] logger: active at %d Hz", LOG_HZ))
            end
        end

        -- Rate-gated sample
        if _log_active and (now_ms - _log_last_ms >= LOG_MS) then
            local dt = math.max((now_ms - _log_last_ms) / 1000, 1e-6)
            _log_last_ms = now_ms

            local current_loc = ahrs:get_location()
            if not current_loc then
                gcs:send_text(4, "[CC] logger: no location — sample skipped")
            else
                local lat_dist = current_loc:get_distance(_log_vtol_loc)
                local lat_vel  = _log_prev_lat_dist and (lat_dist - _log_prev_lat_dist) / dt or 0
                _log_prev_lat_dist = lat_dist

                local height = terrain:height_above_terrain(false)
                if not height then
                    height = current_loc:alt() / 100 - _log_vtol_loc:alt() / 100
                end

                local height_rate = 0
                local vel_ned = ahrs:get_velocity_NED()
                if vel_ned then height_rate = -vel_ned:z() end

                local pitch      = ahrs:get_pitch_rad()
                local gyro       = ahrs:get_gyro()
                local pitch_rate = gyro and gyro:y() or 0

                local m1 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR1))
                local m2 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR2))
                local m3 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR3))
                local m4 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR4))
                local tilt_rad = pwm_to_rad(SRV_Channels:get_output_pwm(FN_TILT),     MAX_TILT_RAD)
                local elev_rad = pwm_to_rad(SRV_Channels:get_output_pwm(FN_ELEVATOR), MAX_ELEV_RAD)

                local dm1_dt   = (m1       - _log_prev_m1)   / dt
                local dm2_dt   = (m2       - _log_prev_m2)   / dt
                local dm3_dt   = (m3       - _log_prev_m3)   / dt
                local dm4_dt   = (m4       - _log_prev_m4)   / dt
                local dtilt_dt = (tilt_rad - _log_prev_tilt) / dt
                local delev_dt = (elev_rad - _log_prev_elev) / dt

                _log_prev_m1, _log_prev_m2, _log_prev_m3, _log_prev_m4 = m1, m2, m3, m4
                _log_prev_tilt, _log_prev_elev = tilt_rad, elev_rad

                local t = #state_log + 1
                state_log[t] = {
                    lat_dist, height,      pitch,
                    lat_vel,  height_rate, pitch_rate,
                    m1, m2, m3, m4, tilt_rad, elev_rad,
                }
                input_rate_log[t] = {
                    dm1_dt, dm2_dt, dm3_dt, dm4_dt, dtilt_dt, delev_dt,
                }

                -- 5-second status
                if t % (LOG_HZ * 5) == 0 then
                    gcs:send_text(7, string.format(
                        "[CC] logger: %d samples | dist=%.1fm h=%.1fm pitch=%.3f tilt=%.3f elev=%.3f",
                        t, lat_dist, height, pitch, tilt_rad, elev_rad
                    ))
                end
            end
        end
    end

    -- ── 3. Controller ───────────────────────────────────────────────────────
    if not start_transition then
        return update, UPDATE_RATE_MS
    end

    if z_flag ~= 1 then
        init_c()
        return update, UPDATE_RATE_MS
    end

    -- Guard: CSV data must be loaded
    if not K or not U_d or not Y_d then
        gcs:send_text(3, "[CC] controller: CSV data missing — cannot run")
        return update, UPDATE_RATE_MS
    end

    -- Guard: need at least one log sample
    local y = state_log[#state_log]
    local u = input_rate_log[#input_rate_log]
    if not y or not u then
        gcs:send_text(4, string.format(
            "[CC] controller: waiting for log data (state_log=%d)", #state_log
        ))
        return update, UPDATE_RATE_MS
    end

    now = millis()
    local t_elapsed = (now - t_0) / 1000

    local y_d = interpolate(Y_d, t_elapsed, "sim")
    local u_d = interpolate(U_d, t_elapsed, "sim")
    local k   = interpolate(K,   t_elapsed, "gain")

    local u1_rpm = pct_to_rpm(u[1], pusher_rpm_min, pusher_rpm_max)
    local u2_rpm = pct_to_rpm(u[2], pusher_rpm_min, pusher_rpm_max)
    local u3_rpm = pct_to_rpm(u[3], pusher_rpm_min, pusher_rpm_max)
    local u4_rpm = pct_to_rpm(u[4], pusher_rpm_min, pusher_rpm_max)
    local u_rpm  = { (u1_rpm + u2_rpm) / 2, (u3_rpm + u4_rpm) / 2, u[3], u[4] }
    if frame_type == "pusher" then
        local u5_rpm = pct_to_rpm(u[5], pusher_rpm_min, pusher_rpm_max)
        u_rpm = { (u1_rpm + u2_rpm) / 2, (u3_rpm + u4_rpm) / 2, u5_rpm, u[4] }
    end

    local y_bar = y     - y_d  --luacheck: ignore
    local u_bar = u_rpm - u_d
    local u_cmd = mat_sub(u_d, mat_mul(k, y))

    -- Rate-limited controller status (1 Hz, shared gate with global status)
    if now - _dbg_last_ms >= 1000 then
        _dbg_last_ms = now
        gcs:send_text(6, string.format(
            "[CC] controller: t=%.3fs | u_cmd[1]=%.2f u_cmd[2]=%.2f u_cmd[3]=%.2f u_cmd[4]=%.2f",
            t_elapsed, u_cmd[1], u_cmd[2], u_cmd[3], u_cmd[4]
        ))
    end

    -- Motors
    if now - update_timestamp_motors > update_interval_motors then
        motor_overide_flag = false
    end
    if now - update_timestamp_motors > update_interval_motors or not motor_overide_flag then
        motor_overide_flag = true
        u1_prev = pct_to_rpm(y[7],  quad_rpm_min, quad_rpm_max)
        u2_prev = pct_to_rpm(y[8],  quad_rpm_min, quad_rpm_max)
        u3_prev = pct_to_rpm(y[9],  quad_rpm_min, quad_rpm_max)
        u4_prev = pct_to_rpm(y[10], quad_rpm_min, quad_rpm_max)
        if frame_type == "pusher" then
            u5_prev = pct_to_rpm(y[11], quad_rpm_min, quad_rpm_max)
        end
        local roll      = ahrs:get_roll()
        local roll_comp = k_roll * roll
        local dt_m      = update_interval_motors / 1000
        u_cmd_1_pwm = Expo(rpm_to_pct(u_cmd[1] * dt_m + u1_prev - roll_comp * math.cos(u_tilt_prev), quad_rpm_min, quad_rpm_max))
        u_cmd_2_pwm = Expo(rpm_to_pct(u_cmd[1] * dt_m + u2_prev + roll_comp * math.cos(u_tilt_prev), quad_rpm_min, quad_rpm_max))
        u_cmd_3_pwm = Expo(rpm_to_pct(u_cmd[2] * dt_m + u3_prev - roll_comp,                         quad_rpm_min, quad_rpm_max))
        u_cmd_4_pwm = Expo(rpm_to_pct(u_cmd[2] * dt_m + u4_prev + roll_comp,                         quad_rpm_min, quad_rpm_max))
        u_cmd_5_pwm = nil
        if frame_type == "pusher" then
            u_cmd_5_pwm = Expo(rpm_to_pct(u_cmd[3] * dt_m + u5_prev, pusher_rpm_min, pusher_rpm_max))
        end
        update_timestamp_motors = millis()
        gcs:send_text(7, string.format(
            "[CC] motors: pwm1=%d pwm2=%d pwm3=%d pwm4=%d",
            u_cmd_1_pwm, u_cmd_2_pwm, u_cmd_3_pwm, u_cmd_4_pwm
        ))
    end
    if motor_overide_flag then
        --SRV_Channels:set_output_pwm(m1_chl, u_cmd_1_pwm)
        --SRV_Channels:set_output_pwm(m2_chl, u_cmd_2_pwm)
        --SRV_Channels:set_output_pwm(m3_chl, u_cmd_3_pwm)
        --SRV_Channels:set_output_pwm(m4_chl, u_cmd_4_pwm)
        if frame_type == "pusher" then
            --SRV_Channels:set_output_pwm(m5_chl, u_cmd_5_pwm)
        end
    end

    -- Elevator
    local elevator_pwm
    if now - update_timestamp_elevator > update_interval_elevator then
        elevator_overide_flag = false
    end
    if now - update_timestamp_elevator > update_interval_elevator or not elevator_overide_flag then
        u_elevator_prev       = y[12]
        elevator_overide_flag = true
        local norm = ((update_interval_elevator / 1000) * u_cmd[4] + u_elevator_prev) / elevator_abs_max
        elevator_pwm = 1500 + norm * 500
        update_timestamp_elevator = millis()
        gcs:send_text(7, string.format("[CC] elevator: pwm=%d", elevator_pwm))
    end
    if elevator_overide_flag then
        SRV_Channels:set_output_pwm(elv_chl, elevator_pwm)
    end

    -- Tilt
    if frame_type == "tilt" then
        if now - update_timestamp_tilt > update_interval_tilt then
            tilt_overide_flag = false
        end
        local tilt_pwm
        if now - update_timestamp_tilt > update_interval_tilt or not tilt_overide_flag then
            u_tilt_prev       = y[11]
            tilt_overide_flag = true
            local norm = ((update_interval_tilt / 1000) * u_cmd[3] + u_tilt_prev) / tilt_max
            tilt_pwm = 1000 + norm * 1000
            update_timestamp_tilt = millis()
            gcs:send_text(7, string.format("[CC] tilt: pwm=%d", tilt_pwm))
        end
        if tilt_overide_flag then
            SRV_Channels:set_output_pwm(tilt_chl,   tilt_pwm)
            SRV_Channels:set_output_pwm(tilt_chl_2, tilt_pwm)
        end
    end

    return update, UPDATE_RATE_MS
end

gcs:send_text(6, "Controller_combine: script loaded, CSV loaded at startup")
return update()
