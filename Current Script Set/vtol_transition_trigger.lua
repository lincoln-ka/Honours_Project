-- vtol_transition_trigger.lua
-- AUTO mode: when the current nav target is VTOL_LAND and distance to it
-- drops to <= TRANSITION_DIST_M, sets the global start_transition = true.

local VTOL_LAND_CMD     = 85    -- MAV_CMD_NAV_VTOL_LAND
local TRANSITION_DIST_M = 300   -- metres (edit as needed)
local UPDATE_RATE_MS    = 100   -- 10 Hz is plenty; 1 ms floods the scheduler
local AUTO_MODE         = 10    -- ArduPlane / QuadPlane AUTO mode

start_transition = false        -- global flag, readable by other scripts
local _triggered     = false
local _last_debug_ms = 0        -- rate-limit GCS debug spam

gcs:send_text(6, "vtol_transition_trigger: script loaded")

-- -------------------------------------------------------------------------
local function update()

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

    return update, UPDATE_RATE_MS
end

return update()
