local file_name = "./scripts/trajec.csv" -- Path to your CSV file on the SD card
local file = io.open(file_name, "r")

gcs:send_text(0, "Before everything")

if not file then
    error("Could not open file: "..file_name)
    return
end

function read_next_target()
    gcs:send_text(0, "1")
    local line = file:read("*line")
    if not line then
        gcs:send_text(0, "2")
        return nil
    end
    gcs:send_text(0, "3")
    

    local values = {}
    for val in string.gmatch(line, '([^,]+)') do
        gcs:send_text(0, "4")
        table.insert(values, tonumber(val))
        gcs:send_text(0, "5")
    end

    return values
end

function update()
    if (arming:is_armed()) then
        gcs:send_text(0, "Vehicle is in guided mode and armed, attempting script run")
        local target = read_next_target()

        if target then
            -- Prepare the SET_POSITION_TARGET_LOCAL_NED message
            
            local msg = vehicle:message_factory():create(
                "SET_POSITION_TARGET_LOCAL_NED", -- Message name
                {
                    type_mask = 0, -- Update all fields
                    coordinate_frame = mavlink.MAV_FRAME_LOCAL_NED,
                    x = target[1], -- X Position
                    y = target[2], -- Y Position
                    z = -target[3], -- Z Position (NED frame so Z is negative)
                    vx = target[7], -- X Velocity
                    vy = target[8], -- Y Velocity
                    vz = -target[9], -- Z Velocity (NED frame so Z is negative)
                    afx = 0, -- X Acceleration (Not specified in your state matrix)
                    afy = 0, -- Y Acceleration (Not specified in your state matrix)
                    afz = 0, -- Z Acceleration (Not specified in your state matrix)
                    yaw = 0, -- Placeholder for Yaw (to be calculated)
                    yaw_rate = target[12] -- Yaw Rate from the state vector
                }
            )
            vehicle:message_send(msg)
        else
            -- Finished reading the file
            return
        end
    end

    return update, 100 -- Update every 100ms (adjust as needed)
end

return update()
