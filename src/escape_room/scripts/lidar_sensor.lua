-- 2D 360° LiDAR sensor for CoppeliaSim.
-- Injected by build_scene.py into a LidarSensor dummy that is a child
-- of the robot's BaseLinkFrame object.
--
-- Uses a Ray-type proximity sensor (created at init, removed at cleanup)
-- rotated per-ray via sim.setObjectMatrix so its local Z-axis points along
-- the desired world angle. sim.checkProximitySensor reads the actual hit.
-- The robot body is made non-detectable by build_scene.py, so rays pass
-- through the chassis and correctly hit walls and obstacles.
-- Scan runs in sysCall_sensing (correct phase for proximity sensor checks).
--
-- Publishes:
--   /scan              (sensor_msgs/msg/LaserScan, 10 Hz, frame laser_link)

-- Explicitly load required modules to resolve warnings
sim = require('sim')
simROS2 = require('simROS2')

local N_RAYS      = 360
local MAX_RANGE   = 5.0
local RANGE_MIN   = 0.05
local RATE_HZ     = 10.0
local LASER_Z     = 0.12       -- sensor height above base_link origin (m)
local LASER_FRAME = 'laser_link'

local scanPub      = nil
local sensorHandle = nil        -- Ray-type proximity sensor (created at init)
local drawHandle   = nil        -- Drawing object for scan visualization
local robotHandle  = nil        -- BaseLinkFrame (pose source)
local rayAngles    = {}
local lastT        = -1e9

function sysCall_init()
    -- Hierarchy: RoboMasterEP / BaseLinkFrame / LidarSensor (this object)
    robotHandle = sim.getObject('..')        -- LidarSensor dummy (script parent; Rz-90 relative to BaseLinkFrame)

    -- Create a Ray-type proximity sensor programmatically.
    sensorHandle = sim.createProximitySensor(
        sim.proximitysensor_ray, -- Correct sensor type
        16,                      -- Subtype placeholder
        1,                      -- Option bit 0 = 1 (Explicitly handled)
        {0, 0, 0, 0, 0, 0, 0, 0},
        {0, MAX_RANGE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} -- Offset 0, Range MAX_RANGE
    )
    sim.setObjectParent(sensorHandle, sim.getObject('.'), true)
    sim.setObjectAlias(sensorHandle, 'LidarRay')

    -- Precompute ray angles (-π … π exclusive endpoint).
    rayAngles = {}
    for i = 0, N_RAYS - 1 do
        rayAngles[i + 1] = -math.pi + i * (2 * math.pi / N_RAYS)
    end

    -- Drawing object: small red dots in world frame, cleared each frame.
    drawHandle = sim.addDrawingObject(
        sim.drawing_points, 4, 0, -1, 99999, {1, 0, 0}
    )

    scanPub = simROS2.createPublisher('/scan', 'sensor_msgs/msg/LaserScan')
end

function sysCall_sensing()
    local t = sim.getSimulationTime()
    if t - lastT < 1.0 / RATE_HZ then return end
    lastT = t

    -- Robot world pose (row-major 3×4 matrix, Lua 1-indexed).
    local mat = sim.getObjectMatrix(robotHandle, -1)
    local rx  = mat[4]
    local ry  = mat[8]
    local rz  = mat[12]
    -- LidarSensor dummy is Rz(-90°) relative to BaseLinkFrame, so +X = physical forward.
    local yaw = math.atan(mat[5], mat[1])
    local oz  = rz + LASER_Z

    -- Clear previous visualization frame.
    sim.addDrawingObjectItem(drawHandle, nil)

    local ranges = {}
    for i, angle in ipairs(rayAngles) do
        local phi = angle + yaw
        local cw  = math.cos(phi)
        local sw  = math.sin(phi)

        sim.setObjectMatrix(sensorHandle, -1, {
            -sw, 0, cw, rx,
             cw, 0, sw, ry,
              0, 1,  0, oz,
        })

        local result, dist = sim.checkProximitySensorEx(
            sensorHandle, sim.handle_all, 1, MAX_RANGE, 0
        )

        if result == 1 then
            ranges[i] = dist
            sim.addDrawingObjectItem(drawHandle, {rx + cw * dist, ry + sw * dist, oz})
        else
            ranges[i] = MAX_RANGE
        end
    end

    -- FIX: Query the native ROS 2 clock instead of using local simulator uptime.
    -- This matches the exact clock used by slam_toolbox and your TF publishers.
    local stamp = simROS2.getTime()
    
    simROS2.publish(scanPub, {
        header          = {stamp = stamp, frame_id = LASER_FRAME},
        angle_min       = -math.pi,
        angle_max       =  math.pi - (2 * math.pi / N_RAYS),
        angle_increment =  2 * math.pi / N_RAYS,
        time_increment  =  0.0,
        scan_time       =  1.0 / RATE_HZ,
        range_min       =  RANGE_MIN,
        range_max       =  MAX_RANGE,
        ranges          =  ranges,
    })

end

function sysCall_cleanup()
    if sensorHandle and sim.isHandle(sensorHandle) then
        sim.removeObjects({sensorHandle})
    end
    if drawHandle then sim.removeDrawingObject(drawHandle) end
    if scanPub    then simROS2.shutdownPublisher(scanPub)  end
end
