-- controller v2

----------------------------------------------------------------------------------
---                            Flight Path Params                              ---
----------------------------------------------------------------------------------
local aspeed_offset=2.00 --desired airspeed during decent above stall air speed (m/s)
local reject_alt=1 --minimum altitude to abort landing attempt (m)
local flare_min_alt=2 --minimum altitude to begin flare (m)
local vel_handover=3 --groundspeed to trigger handoff the arducopter (m/s)

----------------------------------------------------------------------------------

local pitch_d=((-16)/(math.pi*0.5))
local p1=3/(0.5*math.pi)
local d1=5/(0.5*math.pi)


local p=3 --p gain for flare pitch
local d=0 --d gain for flare pitch
local stall = param:get("AIRSPEED_MIN")
local state=0
local og_ang=param:get("Q_TILT_MAX")
-- 0 decent
-- 1 flare
-- 2 handback
-- 3 reject


local triggered =false
local min_trigger_distance=300 --()
local trigger_threshold=5 --threshold of trigger distance


function update()
    
    local mode=quadplane:in_vtol_mode()
    -- if mode then
    --     gcs:send_text(5,"In VTOL Mode")
    -- end
    
    -- gcs:send_text(5,string.format("mode =  %d",mode))
    
    local startpos
    local endpos
    local dist_m=vehicle:get_wp_distance_m()
    local index= mission:get_current_nav_index()
    local type
    if index then
        local item=mission:get_item(index)
        if item then 
            type=item:command()
            -- gcs:send_text(5,string.format("mav cmd = %d",type))
        end
    end

    if dist_m then
        
        -- gcs:send_text(0,string.format("current distance from home is %f",dist_m))
        if dist_m>=min_trigger_distance-trigger_threshold and dist_m<=min_trigger_distance+trigger_threshold and not triggered and type==85 then
            gcs:send_text(5,"trigger controller")
            startpos=ahrs:get_position()
            endpos=ahrs:get_home()
            if startpos and endpos then
                triggered=true
            end
        end
    end

    if triggered then
        local vel=ahrs:get_velocity_NED()
        local ground_speed
        if vel then
            ground_speed=math.sqrt(vel:x()*vel:x()+vel:y()*vel:y())
        end
        local home=ahrs:get_home()
        local pos
        local alt
        if pos then
            pos=ahrs:get_pos()
            alt=pos:alt()/100
        end
        local distance
        local bearing
        if home and pos then
            distance=home:get_distance(pos)
            bearing=(pos:get_bearing(home))
        end
        local pitch_cmd=0


        if state==0 then
            local pitch=ahrs:get_pitch_rad()
            local gyro = ahrs:get_gyro()
            local pr 
            local now
            local pr_cmd
            local last_time=0
            if gyro and pitch then
                pr = gyro:y()
                now=millis()
                if now-last_time>10 then
                    pr_cmd=p1*(pitch_d-pitch)+d1*(-pr)
                end
            end

            if vehicle:get_mode() == 10 then
                if vehicle:nav_scripting_enable(1) then
                     vehicle:set_target_throttle_rpy(50, 0, pr_cmd, 0)
                end
    
            end
            --vehicle:set_target_airspeed(stall+aspeed_offset)
            --param:set("AIRSPEED_CRUISE",stall+aspeed_offset)
            if mode then
                local vel_vec=Vector3f()
                vel_vec:x(10)
                vel_vec:y(0)
                vel_vec:z(4)
                 vehicle:set_target_velocity_NED(vel_vec)
            end
            -- vehicle:set_target_climb_rate(0.1625*(stall+aspeed_offset))
            -- param:set_("Q_TILT_MAX",20)








        end
        if state==1 then
            vehicle:set_target_airspeed(0)
            vehicle:set_target_climb_rate(0)
            if home and pos and ground_speed then
                pitch_cmd=d*(ground_speed)+p*(distance)
                if pitch_cmd>math.pi/2 then
                    pitch_cmd=math.pi/2
                elseif pitch_cmd>math.pi/2 then
                    pitch_cmd=math.pi/2
                end
            end
            vehicle:set_pitch(pitch_cmd)
        end
        if state==2 then
            vehicle:set_mode(23)
            gcs:send_text(0,"Script Excurted Succesfully")
            os.exit()
        end
        if state==3 then
            vehicle:set_mode(23)
            gcs:send_text(0,"Landing Rejected. Returned to Hover")
            os.exit()
        end

        -------------------------------------
        ---     FSM STATE CHANGE LOGIC    ---
        -------------------------------------
        
        if alt and distance and ground_speed then
            if state==0 then
                if alt<reject_alt then
                    state=3
                elseif distance<=5 or alt<flare_min_alt then
                    state=1
                else
                    state=0
                end
            end
            if state==1 then
                if alt<reject_alt then
                    state=3
                elseif ground_speed<=vel_handover or distance<=0.5 then
                    state=2
                else
                    state=1
                end
            end
        end
    end



    return update,1
end


return update()

