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
--   /odom              (nav_msgs/msg/Odometry, 10 Hz, ground-truth pose+vel)
--   odom→base_link TF  (dynamic, every scan tick, ground-truth from CoppeliaSim)

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
local odomPub      = nil
local sensorHandle = nil        -- Ray-type proximity sensor (created at init)
local drawHandle   = nil        -- Drawing object for scan visualization
local robotHandle  = nil        -- BaseLinkFrame (pose source)
local rayAngles    = {}
local lastT        = -1e9

function sysCall_init()
    -- Hierarchy: RoboMasterEP / BaseLinkFrame / LidarSensor (this object)
    robotHandle = sim.getObject('..')        -- BaseLinkFrame

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
    odomPub = simROS2.createPublisher('/odom', 'nav_msgs/msg/Odometry')
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
    -- RoboMaster BaseLinkFrame is -Y-forward in CoppeliaSim.
    -- Extract angle of -Y-axis (physical forward) so scan angle=0 = forward.
    local yaw = math.atan(-mat[6], -mat[2])
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

    -- Broadcast odom→base_link from CoppeliaSim ground-truth pose.
    -- robomaster_ros publishes /odom topic but does not reliably publish
    -- this TF in simulation; without it slam_toolbox and Nav2 have no odom frame.
    local p = sim.getObjectPosition(robotHandle, -1)
    local q = sim.getObjectQuaternion(robotHandle, -1)
    -- CoppeliaSim BaseLinkFrame is -Y-forward; ROS expects X-forward base_link.
    -- Correct by composing q_sim * Rz(-90°) so base_link +X = physical forward.
    local s = math.sqrt(0.5)
    local qrx = s * ( q[1] - q[2])
    local qry = s * ( q[1] + q[2])
    local qrz = s * ( q[3] - q[4])
    local qrw = s * ( q[3] + q[4])
    simROS2.sendTransform({
        header         = {stamp = stamp, frame_id = 'odom'},
        child_frame_id = 'base_link',
        transform      = {
            translation = {x = p[1], y = p[2], z = p[3]},
            rotation    = {x = qrx, y = qry, z = qrz, w = qrw},
        },
    })

    -- Publish /odom topic (nav_msgs/Odometry) — Nav2 requires this in addition to the TF.
    -- Velocity: world-frame from sim, rotated to body frame (base_link).
    local lv, av = sim.getObjectVelocity(robotHandle)
    local cyaw = math.cos(yaw)
    local syaw = math.sin(yaw)
    local vx_body =  cyaw * lv[1] + syaw * lv[2]
    local vy_body = -syaw * lv[1] + cyaw * lv[2]
    local zero36 = {
        0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,
        0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,
    }
    simROS2.publish(odomPub, {
        header         = {stamp = stamp, frame_id = 'odom'},
        child_frame_id = 'base_link',
        pose = {
            pose = {
                position    = {x = p[1], y = p[2], z = p[3]},
                orientation = {x = qrx, y = qry, z = qrz, w = qrw},
            },
            covariance = zero36,
        },
        twist = {
            twist = {
                linear  = {x = vx_body, y = vy_body, z = 0.0},
                angular = {x = 0.0,     y = 0.0,     z = av[3]},
            },
            covariance = zero36,
        },
    })
end

function sysCall_cleanup()
    if sensorHandle and sim.isHandle(sensorHandle) then
        sim.removeObjects({sensorHandle})
    end
    if drawHandle then sim.removeDrawingObject(drawHandle) end
    if scanPub    then simROS2.shutdownPublisher(scanPub)  end
    if odomPub    then simROS2.shutdownPublisher(odomPub) end
end
