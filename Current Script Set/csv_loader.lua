-- csv_loader.lua
-- Watches start_transition (set by vtol_transition_trigger.lua).
-- When the flag fires, loads each configured CSV file into a named global
-- 2-D array readable by any other loaded Lua script.
--
-- Place all CSV files in the scripts/ folder alongside this script.
-- In SITL the working directory is the SITL root, so paths begin with "scripts/".
-- On real hardware they live on the SD card under APM/scripts/.

-- ── Configuration ────────────────────────────────────────────────────────────
-- Each entry: { file = "filename_without_extension", global = "GlobalArrayName" }
-- The loaded data is written to _ENV[global] so other scripts can access it by name.

local CSV_DIR = "scripts/"

local FILES = {
    { file = "K_test", global = "K" },
    { file = "U", global = "U_d" },
    { file = "Y", global = "Y_d" },
}

-- ── Module state ──────────────────────────────────────────────────────────────

local UPDATE_RATE_MS = 200
local _loaded = false

-- Pre-declare all globals as nil so other scripts see them immediately.
for _, entry in ipairs(FILES) do
    _ENV[entry.global] = nil
end

-- ── Helpers ───────────────────────────────────────────────────────────────────

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

-- ── Main update loop ──────────────────────────────────────────────────────────

local function update()

    if not start_transition then
        return update, UPDATE_RATE_MS
    end

    if _loaded then
        return update, UPDATE_RATE_MS
    end

    gcs:send_text(6, "csv_loader: start_transition detected, loading CSV files...")

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

    _loaded = true
    gcs:send_text(6, "csv_loader: all files processed")
    return update, UPDATE_RATE_MS
end

gcs:send_text(6, "csv_loader: script loaded")
return update()
