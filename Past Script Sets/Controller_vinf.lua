-- inputs of Y_d,U_d,Y,U,K

--input params
local sim_time_total=17.7681928636437
local sim_time_steps=33
local frame_type="tilt";
    --type == tilt:quadtillt
    --type == pusher:quad+pusher/puller
local gain_td=0.1 



local sim_td=sim_time_total/sim_time_steps

-- local log_time_total
-- local log_time_steps
local log_td=10

-- channel setup
local m1_chl=1
local m2_chl=2
local m3_chl=3
local m4_chl=4
local tilt_chl=5
local elv_chl=6


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

-- set time zero for indexing states and inputs
local function init ()
    if z_flag~=1 then
        t_0=millis()
        z_flag=1
    end
end
init()

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
local function interpolate (func,now_p,dataset)
    local x1
    local x3
    if (dataset=="gain") then
        x1=math.floor(now_p/gain_td)
        x3=now_p/gain_td
    elseif (dataset=="sim") then
        x1=math.floor(now_p/sim_td)
        x3=now_p/sim_td
    elseif (dataset=="log") then
        x1=math.floor(now_p/log_td)
        x3=now_p/log_td
    end
    local x2=x1+1
    local w1=x3-x1
    local w2=x2-x3
    local y3=w2*func[x1]+w1*func[x2]
    return y3
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

local function update ()
    if (start_transition==true)then
        init()
    end
    if (z_flag==1) then
        now=millis()

        --interpolation of inputs
        local y_d=interpolate(Y_d,now,sim)
        local u_d=interpolate(U_d,now,sim)
        local y=interpolate(Y,now,log)
        local u=interpolate(U,now,log)
        local k=interpolate(K,now,gain)

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
            SRV_Channels:set_output_pwm(m1_chl, u_cmd_1_pwm)
            SRV_Channels:set_output_pwm(m1_chl, u_cmd_2_pwm)
            SRV_Channels:set_output_pwm(m3_chl, u_cmd_3_pwm)
            SRV_Channels:set_output_pwm(m4_chl, u_cmd_4_pwm)
            if (frame_type=="pusher") then
                SRV_Channels:set_output_pwm(m5_chl, u_cmd_5_pwm)
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
            SRV_Channels:set_output_pwm(elv_chl, elevator_pwm)
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
                SRV_Channels:set_output_pwm(tilt_chl, tilt_pwm)
            end
        end

    end
    return update,update_rate
end
return update()