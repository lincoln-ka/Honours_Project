------------------------------------------------------------------
---                      Parameters                            ---
------------------------------------------------------------------
local update_rate=10
local up0=false
local tr0=false

--controller
local sim_time_total=17.7681928636437
local sim_time_steps=33
local frame_type="tilt";
    --type == tilt:quadtillt
    --type == pusher:quad+pusher/puller
local gain_td=0.1 
local sim_td=sim_time_total/sim_time_steps
local log_td=10

-- channel setup
local m1_chl=6
local m2_chl=4
local m3_chl=5
local m4_chl=7
local tilt_chl=8
local tilt_chl_2=9
local elv_chl=1


local quad_rpm_min=0
local quad_rpm_max=10000
local pusher_rpm_min=0
local pusher_rpm_max=10000
local motor_overide_flag=false
local u_cmd_1_pwm
local u_cmd_2_pwm
local u_cmd_3_pwm
local u_cmd_4_pwm
local u_cmd_5_pwm
local k_roll=0.2
local expo=20*180/math.pi
local elevator_overide_flag=false
local elevator_abs_max=math.pi/4
local tilt_overide_flag=false
local tilt_max=math.pi/2
local u1_prev
local u2_prev
local u3_prev
local u4_prev
local u5_prev
local u_elevator_prev
local u_tilt_prev
local t_0=0
local z_flag
local now
local update_interval_motors=3.3 --300 ish hz
local update_interval_elevator=100 --10hz
local update_interval_tilt=100 --10hz
local update_timestamp_motors=0
local update_timestamp_elevator=0
local update_timestamp_tilt=0
local update_rate=10



------------------------------------------------------------------
---                      Setup                                 ---
------------------------------------------------------------------



--- CSV LOADER SETUP
local CSV_DIR = "scripts/"

local FILES = {
    { file = "K", global = "K" },
    { file = "U", global = "U_d" },
    { file = "Y", global = "Y_d" },
}

-- Pre-declare all globals as nil so other scripts see them immediately.
for _, entry in ipairs(FILES) do
    _ENV[entry.global] = nil
end

--- Trigger
local VTOL_LAND_CMD     = 85    -- MAV_CMD_NAV_VTOL_LAND
local TRANSITION_DIST_M = 300   -- metres (edit as needed)
local UPDATE_RATE_MS    = 100   -- 10 Hz is plenty; 1 ms floods the scheduler
local AUTO_MODE         = 10    -- ArduPlane / QuadPlane AUTO mode

start_transition = false        -- global flag, readable by other scripts
local _triggered     = false
local _last_debug_ms = 0        -- rate-limit GCS debug spam

--- State observer
local FN_MOTOR1    = 33   -- SRV_Channel::k_motor1
local FN_MOTOR2    = 34
local FN_MOTOR3    = 35
local FN_MOTOR4    = 36
local FN_TILT      = 41   -- k_tiltMotor
local FN_ELEVATOR  = 19   -- k_elevator
local MAX_TILT_RAD = 1.5708   -- ~90 deg
local MAX_ELEV_RAD = 0.4363   -- ~25 deg

local _obs_prev_m1, _obs_prev_m2, _obs_prev_m3, _obs_prev_m4 = 0, 0, 0, 0
local _obs_prev_tilt, _obs_prev_elev = 0, 0
local _obs_prev_lat_dist = nil
local _obs_last_ms  = 0
local _obs_vtol_loc = nil


------------------------------------------------------------------
---                      Functions                             ---
------------------------------------------------------------------

-- state observer helpers
local function pwm_to_throttle_pct(pwm)
    if not pwm or pwm < 900 then return 0 end
    return math.max(0, math.min(100, (pwm - 1000) / 10))
end

local function pwm_to_rad(pwm, max_rad)
    if not pwm or pwm < 900 then return 0 end
    return (pwm - 1500) / 500 * max_rad
end

-- Returns y_now (12x1) and u_now (6x1) of the current aircraft state.
-- y_now: { lat_dist, height, pitch, lat_vel, height_rate, pitch_rate,
--           m1, m2, m3, m4, tilt_rad, elev_rad }
-- u_now: { dm1_dt, dm2_dt, dm3_dt, dm4_dt, dtilt_dt, delev_dt }
local function get_current_state()
    local now_ms = millis()
    local dt = math.max((now_ms - _obs_last_ms) / 1000, 1e-6)
    _obs_last_ms = now_ms

    -- Cache VTOL_LAND location once available
    if not _obs_vtol_loc then
        local idx = mission:get_current_nav_index()
        if idx then
            local item = mission:get_item(idx)
            if item and item:command() == 85 then
                local base = ahrs:get_location()
                if base then
                    base:lat(item:x())
                    base:lng(item:y())
                    base:alt(math.floor(item:z() * 100))
                    _obs_vtol_loc = base
                end
            end
        end
    end

    -- Lateral distance and velocity
    local lat_dist, lat_vel = 0, 0
    local current_loc = ahrs:get_location()
    if current_loc and _obs_vtol_loc then
        lat_dist = current_loc:get_distance(_obs_vtol_loc)
        lat_vel  = _obs_prev_lat_dist and (lat_dist - _obs_prev_lat_dist) / dt or 0
        _obs_prev_lat_dist = lat_dist
    end

    -- Height above ground
    local height = 0
    if current_loc then
        height = terrain:height_above_terrain(false)
        if not height then
            local land_alt = _obs_vtol_loc and _obs_vtol_loc:alt() / 100 or 0
            height = current_loc:alt() / 100 - land_alt
        end
    end

    -- Height rate
    local height_rate = 0
    local vel_ned = ahrs:get_velocity_NED()
    if vel_ned then height_rate = -vel_ned:z() end

    -- Attitude
    local pitch      = ahrs:get_pitch_rad() or 0
    local gyro       = ahrs:get_gyro()
    local pitch_rate = gyro and gyro:y() or 0

    -- Motor throttle (%)
    local m1 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR1))
    local m2 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR2))
    local m3 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR3))
    local m4 = pwm_to_throttle_pct(SRV_Channels:get_output_pwm(FN_MOTOR4))

    -- Servo angles (rad)
    local tilt_rad = pwm_to_rad(SRV_Channels:get_output_pwm(FN_TILT),     MAX_TILT_RAD)
    local elev_rad = pwm_to_rad(SRV_Channels:get_output_pwm(FN_ELEVATOR), MAX_ELEV_RAD)

    -- Rates of change
    local dm1_dt   = (m1       - _obs_prev_m1)   / dt
    local dm2_dt   = (m2       - _obs_prev_m2)   / dt
    local dm3_dt   = (m3       - _obs_prev_m3)   / dt
    local dm4_dt   = (m4       - _obs_prev_m4)   / dt
    local dtilt_dt = (tilt_rad - _obs_prev_tilt) / dt
    local delev_dt = (elev_rad - _obs_prev_elev) / dt

    _obs_prev_m1, _obs_prev_m2, _obs_prev_m3, _obs_prev_m4 = m1, m2, m3, m4
    _obs_prev_tilt, _obs_prev_elev = tilt_rad, elev_rad

    local y_now = {
        lat_dist,    -- [1]  lateral distance to VTOL_LAND WP (m)
        height,      -- [2]  height above ground (m)
        pitch,       -- [3]  pitch angle (rad)
        lat_vel,     -- [4]  lateral velocity (m/s)
        height_rate, -- [5]  height rate (m/s)
        pitch_rate,  -- [6]  pitch rate (rad/s)
        m1,          -- [7]  motor 1 throttle (%)
        m2,          -- [8]  motor 2 throttle (%)
        m3,          -- [9]  motor 3 throttle (%)
        m4,          -- [10] motor 4 throttle (%)
        tilt_rad,    -- [11] tilt servo (rad)
        elev_rad,    -- [12] elevator (rad)
    }

    local u_now = {
        dm1_dt,   -- [1]  motor 1 rate (%/s)
        dm2_dt,   -- [2]  motor 2 rate (%/s)
        dm3_dt,   -- [3]  motor 3 rate (%/s)
        dm4_dt,   -- [4]  motor 4 rate (%/s)
        dtilt_dt, -- [5]  tilt rate (rad/s)
        delev_dt, -- [6]  elevator rate (rad/s)
    }

    return y_now, u_now
end

--csv
local function parse_csv(filepath)
    local file = io.open(filepath, "r")
    if not file then
        gcs:send_text(3, "csv_loader: cannot open " .. filepath)
        return nil
    end

    local data = {}
    for line in file:lines() do
        if line ~= "" then
            local row = {}
            for field in line:gmatch("[^,]+") do
                local num = tonumber(field)
                if num then
                    row[#row + 1] = num
                end
            end
            if #row > 0 then
                data[#data + 1] = row
            end
        end
    end

    file:close()
    return data
end

local function csv_loader()
    gcs:send_text(6, "csv_loader: loading CSV files...")

    for _, entry in ipairs(FILES) do
        local path   = CSV_DIR .. entry.file .. ".csv"
        local result = parse_csv(path)
        if result then
            _ENV[entry.global] = result
            gcs:send_text(6, string.format(
                "csv_loader: %s -> %s (%d rows x %d cols)",
                entry.file, entry.global, #result, #result[1]
            ))
        else
            gcs:send_text(3, "csv_loader: " .. entry.file .. " failed to load")
        end
    end

    gcs:send_text(6, "csv_loader: all files processed")
end

-- trigger
local function controller_trigger()
    -- Only act in AUTO mode; reset state when leaving AUTO.
    if vehicle:get_mode() ~= AUTO_MODE then
        if _triggered then
            _triggered       = false
            start_transition = false
        end
        return update, UPDATE_RATE_MS
    end

    -- Index of the waypoint currently being navigated to.
    local current_idx = mission:get_current_nav_index()
    if not current_idx then
        return update, UPDATE_RATE_MS
    end

    -- Fetch that mission item and check its command ID.
    local nav_item = mission:get_item(current_idx)
    if not nav_item then
        return update, UPDATE_RATE_MS
    end

    if nav_item:command() ~= VTOL_LAND_CMD then
        -- Not flying toward a VTOL_LAND; clear trigger so it can fire again.
        if _triggered then
            _triggered       = false
            start_transition = false
        end
        return update, UPDATE_RATE_MS
    end

    -- Flag already set for this approach.
    if _triggered then
        return update, UPDATE_RATE_MS
    end

    -- Current vehicle position.
    local current_loc = ahrs:get_location()
    if not current_loc then
        return update, UPDATE_RATE_MS
    end

    -- Build the VTOL_LAND location. Re-use ahrs:get_location() as a base
    -- object (avoids Location() constructor and :copy() compatibility issues)
    -- then overwrite lat/lng with the mission item coordinates.
    local vtol_loc = ahrs:get_location()
    if not vtol_loc then
        return update, UPDATE_RATE_MS
    end
    vtol_loc:lat(nav_item:x())   -- int32, degrees * 1e7
    vtol_loc:lng(nav_item:y())   -- int32, degrees * 1e7

    -- 2-D distance in metres.
    local dist_m = current_loc:get_distance(vtol_loc)

    -- Rate-limited debug: print at most once per second.
    local now_ms = millis()
    if now_ms - _last_debug_ms >= 1000 then
        _last_debug_ms = now_ms
        gcs:send_text(7, string.format(
            "VTOL_LAND WP %d: dist=%.1f m  threshold=%d m",
            current_idx, dist_m, TRANSITION_DIST_M
        ))
    end

    -- Trigger once when the distance crosses the threshold.
    if dist_m <= TRANSITION_DIST_M then
        start_transition = true
        _triggered       = true
        gcs:send_text(6, string.format(
            "start_transition SET at %.1f m from VTOL_LAND WP %d",
            dist_m, current_idx
        ))
    end
end

-- controller

local function controller_init ()
    t_0=millis()
    tr0=true
end

-- matrix subtraction: C = A - B  (must be same dimensions)
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

-- matrix multiplication: C = A * B  (A is m×k, B is k×n)
local function mat_mul(A, B)
    local m, k, n = #A, #B, #B[1]
    local C = {}
    for i = 1, m do
        C[i] = {}
        for j = 1, n do
            local s = 0
            for p = 1, k do
                s = s + A[i][p] * B[p][j]
            end
            C[i][j] = s
        end
    end
    return C
end

-- interpolates matlab arrays to current time
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
    else
        gcs:send_text(3, "interpolate: unknown dataset '" .. tostring(dataset) .. "'")
        return nil
    end

    x1 = math.max(1, x1)                    -- clamp low: Lua arrays are 1-indexed
    local x2 = math.min(x1 + 1, #func)      -- clamp high: don't exceed array length
    local w1  = x3 - math.floor(x3)         -- fractional part
    local w2  = 1 - w1
    local row1, row2 = func[x1], func[x2]
    local y = {}
    for i = 1, #row1 do
        y[i] = w2 * row1[i] + w1 * row2[i]
    end
    return y
end

--rearranges matlab arrays to correct dimensions for script
--decided to do this in matlab pre-export

--throttle expo (scalar)
local function Expo(thr_pct)
    local thr_norm=thr_pct/100
    local thr_expo=thr_norm^3*expo+thr_norm*(1-expo)
    local thr_expo_pwm=1000+thr_expo*1000
    return thr_expo_pwm
end

--converts throttle values from rpm to %
local function rpm_to_pct (rpm,min,max)
    local pct=(rpm-min)/(max-min)
    return pct
end

--converts throttle values from %(expo) to rpm 
-- VALUES MUST BE CHANGED FOR NEW EXPO VALUE REFER TO EXCEL SHEET
local function pct_to_rpm (thr_pct,min_rpm,max_rpm)
    local norm_e=thr_pct/100
    local norm=0.6826*(norm_e^3)-1.7606*(norm_e^2)+2.0796*norm_e+0.0011
    local rpm=norm*(max_rpm-min_rpm)
    return rpm
end


local function controller()
    now=millis()

        --interpolation of inputs
        local y_d=interpolate(Y_d,now,"sim")
        local u_d=interpolate(U_d,now,"sim")
        local y=interpolate(Y,now,"log")
        local u=interpolate(U,now,"log")
        local k=interpolate(K,now,"gain")

        if not y or not u or not k then
            gcs:send_text(3,"controller: interpolation returned nil — skipping")
            return update,update_rate
        end

        --pct/s to rpm/s
        local u1_rpm=pct_to_rpm(u[1])
        local u2_rpm=pct_to_rpm(u[2])
        local u3_rpm=pct_to_rpm(u[3])
        local u4_rpm=pct_to_rpm(u[4])
        local u5_rpm=nil
        local u_rpm={(u1_rpm+u2_rpm)/2,(u3_rpm+u4_rpm)/2,u[3],u[4]}
        if (frame_type=="pusher") then
            local u5_rpm=pct_to_rpm(u[5])
            u_rpm={(u1_rpm+u2_rpm)/2,(u3_rpm+u4_rpm)/2,u5_rpm,u[4]}
        end

        u_rpm={(u1_rpm+u2_rpm)/2,(u3_rpm+u4_rpm)/2,u[3],u[4]}

        --error calculation
        local y_bar=y-y_d
        local u_bar=u_rpm-u_d

        -- cmd vector calculation
        --u_cmd=u_d-k*u_bar
        local u_cmd=mat_sub(u_d,mat_mul(k,u_bar))


        --motor
        --stop motor overide if command is not updated
        if (now-update_timestamp_motors>update_interval_motors) then
            motor_overide_flag=false
        end

        --motor command calculate
        if (now-update_timestamp_motors>update_interval_motors or motor_overide_flag==false) then
            --cmd rpm --> pct --> norm--> expo --> pwm 
            motor_overide_flag=true
            --update current motor values
            u1_prev=pct_to_rpm(y[7],quad_rpm_min,quad_rpm_max)
            u2_prev=pct_to_rpm(y[8],quad_rpm_min,quad_rpm_max)
            u3_prev=pct_to_rpm(y[9],quad_rpm_min,quad_rpm_max)
            u4_prev=pct_to_rpm(y[10],quad_rpm_min,quad_rpm_max)
            if (frame_type=="pusher") then
                u5_prev=pct_to_rpm(y[11],quad_rpm_min,quad_rpm_max)
            end
            local roll =ahrs:get_roll() --rads
            local roll_compensation_rpm=k_roll*roll

            --update pwm values
            u_cmd_1_pwm=Expo(rpm_to_pct((u_cmd[1]*(update_interval_motors/1000))+u1_prev-roll_compensation_rpm*math.cos(u_tilt_prev),quad_rpm_min,quad_rpm_max))
            u_cmd_2_pwm=Expo(rpm_to_pct((u_cmd[1]*(update_interval_motors/1000))+u2_prev+roll_compensation_rpm*math.cos(u_tilt_prev),quad_rpm_min,quad_rpm_max))
            u_cmd_3_pwm=Expo(rpm_to_pct((u_cmd[2]*(update_interval_motors/1000))+u3_prev-roll_compensation_rpm,quad_rpm_min,quad_rpm_max))
            u_cmd_4_pwm=Expo(rpm_to_pct((u_cmd[2]*(update_interval_motors/1000))+u4_prev+roll_compensation_rpm,quad_rpm_min,quad_rpm_max))
            u_cmd_5_pwm=nil
            if (frame_type=="pusher") then
                u_cmd_5_pwm=Expo(rpm_to_pct((u_cmd[3]*(update_interval_motors/1000))+u5_prev,pusher_rpm_min,pusher_rpm_max))
            end
        end

        --motor overide
        if (motor_overide_flag) then
            -- overide output 
            SRV_Channels:set_output_pwm_chan_timeout(m1_chl, u_cmd_1_pwm,update_rate)
            SRV_Channels:set_output_pwm_chan_timeout(m2_chl, u_cmd_2_pwm,update_rate)
            SRV_Channels:set_output_pwm_chan_timeout(m3_chl, u_cmd_3_pwm,update_rate)
            SRV_Channels:set_output_pwm_chan_timeout(m4_chl, u_cmd_4_pwm,update_rate)
            if (frame_type=="pusher") then
                SRV_Channels:set_output_pwm_chan_timeout(m5_chl, u_cmd_5_pwm,update_rate)
            end
        end
       

        --elevator
        local elevator_pwm
        
        -- stop overiding elevator if too long without update
        if (now-update_timestamp_elevator>update_interval_elevator) then
            elevator_overide_flag=false
        end

        -- calculate new elevator value
        if (now-update_timestamp_elevator>update_interval_elevator or elevator_overide_flag==false) then
            -- update current elevator value
            u_elevator_prev=y[12]

            --calculate pwm value
            elevator_overide_flag=true
            local u_cmd_elevator_norm=((update_interval_elevator/1000)*u_cmd[4]+u_elevator_prev)/elevator_abs_max
            local elevator_pwm=1500+u_cmd_elevator_norm*500
        end

        -- overide elevator
        if (elevator_overide_flag) then
            SRV_Channels:set_output_pwm_chan_timeout(elv_chl, elevator_pwm,update_rate)
        end


        --tilt
        if (frame_type=="tilt") then
            
            --stop overide if tilt command not updated
            if (now-update_timestamp_tilt>update_interval_tilt) then
                tilt_overide_flag=false
            end

            --calculate new tilt command
            local tilt_pwm
            if (now-update_timestamp_tilt>update_interval_tilt or tilt_overide_flag==false) then
                --update current value
                u_tilt_prev=y[11]
                
                --update pwm value
                elevator_overide_flag=true
                local u_cmd_tilt_norm=((update_interval_tilt/1000)*u_cmd[3]+u_tilt_prev)/tilt_max
                tilt_pwm=1000+u_cmd_tilt_norm*1000
            end

            --overide tilt command
            if (tilt_overide_flag) then
                SRV_Channels:set_output_pwm_chan_timeout(tilt_chl, tilt_pwm,update_rate)
                SRV_Channels:set_output_pwm_chan_timeout(tilt_chl_2, tilt_pwm,update_rate)
            end
        end

end

------------------------------------------------------------------
---                      MAIN                                  ---
------------------------------------------------------------------



local function init ()
    csv_loader()
    up0=true
end


local function update()

    if (up0==false) then
        init()
    end
    -- if (up0==true) then
    --     gcs:send_text(1, "init ran")
    -- end

    if (_triggered==false) then
        controller_trigger()
    end

    if (_triggered==true) then
        if (tr0==false) then
            controller_init()
        end
        local y, u = get_current_state()
        controller()
    end
   

    --SRV_Channels:set_output_pwm_chan_timeout(1, 2000,update_rate)

    return update,update_rate
end

return update()