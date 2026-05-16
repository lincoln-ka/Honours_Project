-- csv_loader.lua
-- Watches start_transition (set by vtol_transition_trigger.lua).
-- When the flag fires, loads all five CSV files into global 2-D arrays:
--   A_test[row][col], B_test[row][col], S_test[row][col],
--   U_test[row][col], Y_test[row][col]
--
-- Place all CSV files in the scripts/ folder alongside this script.
-- In SITL the working directory is the SITL root, so paths begin with "scripts/".
-- On real hardware they live on the SD card under APM/scripts/.

local CSV_DIR = "scripts/"

local UPDATE_RATE_MS = 200

local _loaded = false   -- only load once per session

-- Global arrays written here; readable by any other loaded Lua script.
A_test = nil
B_test = nil
S_test = nil
U_test = nil
Y_test = nil

-- -------------------------------------------------------------------------
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

local function load_and_report(name)
    local result = parse_csv(CSV_DIR .. name .. ".csv")
    if result then
        gcs:send_text(6, string.format(
            "csv_loader: %s loaded — %d rows x %d cols",
            name, #result, #result[1]
        ))
    else
        gcs:send_text(3, "csv_loader: " .. name .. " failed to load")
    end
    return result
end

-- -------------------------------------------------------------------------
local function update()

    -- Wait until the transition flag is set by vtol_transition_trigger.lua.
    if not start_transition then
        return update, UPDATE_RATE_MS
    end

    -- Only load once.
    if _loaded then
        return update, UPDATE_RATE_MS
    end

    gcs:send_text(6, "csv_loader: start_transition detected, loading CSV files...")

    A_test = load_and_report("A_test")
    B_test = load_and_report("B_test")
    S_test = load_and_report("S_test")
    U_test = load_and_report("U_test")
    Y_test = load_and_report("Y_test")

    _loaded = true
    gcs:send_text(6, "csv_loader: all files processed")
    return update, UPDATE_RATE_MS
end

gcs:send_text(6, "csv_loader: script loaded")
return update()
