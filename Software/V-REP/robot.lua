-- Lua script for the robot simulated in V-REP and controlled from ROS
-- Author: Karl Kangur <karl.kangur@gmail.com>

function speedMessage_callback(msg)
    -- Receieve and interpret the Twist message for robot control
    -- Reference: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html

    -- Values are in meters
    local wheel_radius = 0.035
    local wheel_separation = 0.1056

    -- Transform the cmd_vel into the left and right wheel speeds depending on the wheel geometry for differential drive
    local speed_left = msg['linear']['x'] / wheel_radius + msg['angular']['z'] * wheel_separation / wheel_radius
    local speed_right = msg['linear']['x'] / wheel_radius - msg['angular']['z'] * wheel_separation / wheel_radius

    -- Apply the speeds to the joints
    simSetJointTargetVelocity(joint_drive_left, speed_left)
    simSetJointTargetVelocity(joint_drive_right, speed_right)
end

if (sim_call_type == sim_childscriptcall_initialization) then
    -- Greeting message
    simAddStatusbarMessage('Starting robot simulation')

    -- Define constants for the simulation
    -- Offset between the body and the ground plane (in meters)
    body_origin_height = 0.047
    -- Offset between the center of the body and the front IR sensors (in meters)
    ir_offset_from_center = 0.06176
    -- Maximum IR sensing distance (in meters)
    max_ir_value = 0.2

    -- Simulation time publisher
    clock_pub = simExtRosInterface_advertise('/clock', 'rosgraph_msgs/Clock')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(clock_pub)

    -- Get camera handler
    linear_camera = simGetObjectHandle('linear_camera')

    -- Linear camera publisher
    linear_camera_pub = simExtRosInterface_advertise('/linear_camera', 'sensor_msgs/Image')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(linear_camera_pub)

    -- Get the joint handlers
    joint_drive_left = simGetObjectHandle('joint_drive_left')
    joint_drive_right = simGetObjectHandle('joint_drive_right')

    -- Joint state (position and velocity) publisher
    joints_pub = simExtRosInterface_advertise('/joint_states', 'sensor_msgs/JointState')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(joints_pub)

    -- Get the infra-red sensor handlers
    ir_front_left = simGetObjectHandle('ir_front_left')
    ir_front_left_center = simGetObjectHandle('ir_front_left_center')
    ir_front_right = simGetObjectHandle('ir_front_right')
    ir_front_right_center = simGetObjectHandle('ir_front_right_center')
    ir_under_left = simGetObjectHandle('ir_under_left')
    ir_under_right = simGetObjectHandle('ir_under_right')

    -- Range publishers for IR sensors
    ir_front_left_pub = simExtRosInterface_advertise('/ir_front_left', 'sensor_msgs/Range')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(ir_front_left_pub)
    ir_front_left_center_pub = simExtRosInterface_advertise('/ir_front_left_center', 'sensor_msgs/Range')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(ir_front_left_center_pub)
    ir_front_right_pub = simExtRosInterface_advertise('/ir_front_right', 'sensor_msgs/Range')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(ir_front_right_pub)
    ir_front_right_center_pub = simExtRosInterface_advertise('/ir_front_right_center', 'sensor_msgs/Range')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(ir_front_right_center_pub)

    -- IR sensors under the robot, publish as illuminance sensors
    ir_under_left_pub = simExtRosInterface_advertise('/ir_under_left', 'sensor_msgs/Illuminance')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(ir_under_left_pub)
    ir_under_right_pub = simExtRosInterface_advertise('/ir_under_right', 'sensor_msgs/Illuminance')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(ir_under_right_pub)

    -- Make the point cloud publisher, will actually use the IR sensors
    -- The LaserScan topic could also be used, but the rays are not equally spaced and it makes it akward,
    -- so rather than having a non-perfect LaserScan topic a perfect PointCloud is later transformed to LaserScan
    cloud_pub = simExtRosInterface_advertise('/pointcloud', 'sensor_msgs/PointCloud2')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(cloud_pub)

    -- Get the body handler, needed to get the absolute position and velocity of the robot in the scene
    robot = simGetObjectHandle('robot')

    -- Odomety publisher, useful for mapping
    odom_pub = simExtRosInterface_advertise('/odom', 'nav_msgs/Odometry')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(odom_pub)

    -- Subscribe to Twist topics coming to the /cmd_vel input, call the callback every time there is a new message
    speed_sub = simExtRosInterface_subscribe('/cmd_vel', 'geometry_msgs/Twist', 'speedMessage_callback')
    simExtRosInterface_subscriberTreatUInt8ArrayAsString(speed_sub)
end

if (sim_call_type == sim_childscriptcall_sensing) then
    -- Get the simulation time stamp
    local frame_stamp = simGetSystemTime()

    -- Get the position of the joints
    local position_left = simGetJointPosition(joint_drive_left)
    local position_right = simGetJointPosition(joint_drive_right)

    -- Get the velocity of the joints
    local velocity_left = simGetJointTargetVelocity(joint_drive_left)
    local velocity_right = simGetJointTargetVelocity(joint_drive_right)

    -- Build the sensor_msgs/JointState type message
    -- Reference: http://docs.ros.org/jade/api/sensor_msgs/html/msg/JointState.html
    local joints_data = {}
    joints_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "base_link"}
    joints_data['name'] = {"joint_left", "joint_right"}
    joints_data['position'] = {position_left, position_right}
    joints_data['velocity'] = {velocity_left, velocity_right}
    joints_data['effort'] = {}
    simExtRosInterface_publish(joints_pub, joints_data)

    -- Get the robot position, orientation and velocities
    local position = simGetObjectPosition(robot, -1)
    local rotation = simGetObjectQuaternion(robot, -1)
    local velocity_linear, velocity_angular = simGetObjectVelocity(robot)

    -- Send the transform between the odom and base_link frame, defines the absolute position of the robot in the scene
    -- Reference: http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/TransformStamped.html
    local base_data = {}
    base_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "odom"}
    base_data['child_frame_id'] = "base_link"
    base_data['transform'] = {}
    base_data['transform']['translation'] = {x = position[1], y = position[2], z = position[3] - body_origin_height}
    base_data['transform']['rotation'] = {x = rotation[1], y = rotation[2], z = rotation[3], w = rotation[4]}
    simExtRosInterface_sendTransform(base_data)

    -- Odometry message is necessary for the navigation on a map
    -- Reference: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
    local odom_data = {}
    odom_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "odom"}
    odom_data['child_frame_id'] = "base_link"
    odom_data['pose'] = {}
    odom_data['pose']['pose'] = {}
    odom_data['pose']['pose']['position'] = {x = position[1], y = position[2], z = position[3] - body_origin_height}
    odom_data['pose']['pose']['orientation'] = {x = rotation[1], y = rotation[2], z = rotation[3], w = rotation[4]}
    odom_data['pose']['covariance'] = {0}
    odom_data['twist'] = {}
    odom_data['twist']['twist'] = {}
    odom_data['twist']['twist']['linear'] = {x = velocity_linear[1], y = velocity_linear[2], z = velocity_linear[3]}
    odom_data['twist']['twist']['angular'] = {x = velocity_angular[1], y = velocity_angular[2], z = velocity_angular[3]}
    odom_data['twist']['covariance'] = {0}
    simExtRosInterface_publish(odom_pub, odom_data)

    -- Publish the image of the linear camera
    -- Reference: http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetVisionSensorCharImage.htm
    local linear_camera_image, linear_camera_w, linear_camera_h = simGetVisionSensorCharImage(linear_camera)
    local linear_camera_data={}
    linear_camera_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "linear_camera"}
    linear_camera_data['height'] = linear_camera_h
    linear_camera_data['width'] = linear_camera_w
    linear_camera_data['encoding'] = 'rgb8'
    linear_camera_data['is_bigendian'] = 1
    linear_camera_data['step'] = linear_camera_data['width']*3
    linear_camera_data['data'] = linear_camera_image
    simExtRosInterface_publish(linear_camera_pub, linear_camera_data)

    -- Publish the image of the camera
    -- Reference: http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetVisionSensorCharImage.htm
    -- simGetVisionSensorCharImage() returns a byte array when the last parameter is 1, it must be unpacked first
    local ir_under_left_illuminance = simUnpackBytes(simGetVisionSensorCharImage(ir_under_left, 0, 0, 1, 1, 1))
    local ir_under_left_data={}
    ir_under_left_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_under_left"}
    -- Send the first pixel of the 3 as they are all the same
    ir_under_left_data['illuminance'] = ir_under_left_illuminance[1]
    ir_under_left_data['variance'] = 0.0
    simExtRosInterface_publish(ir_under_left_pub, ir_under_left_data)

    local ir_under_right_illuminance = simUnpackBytes(simGetVisionSensorCharImage(ir_under_right, 0, 0, 1, 1, 1))
    local ir_under_right_data={}
    ir_under_right_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_under_right"}
    -- Send the first pixel of the 3 as they are all the same
    ir_under_right_data['illuminance'] = ir_under_right_illuminance[1]
    ir_under_right_data['variance'] = 0.0
    simExtRosInterface_publish(ir_under_right_pub, ir_under_right_data)

    -- Get the IR sensor values
    -- When the distance sensor does not detect anything it returns "nil", which is annoying, transform them to max_ir_value
    result, ir_front_left_distance = simReadProximitySensor(ir_front_left)
    if not ir_front_left_distance then
        ir_front_left_distance = max_ir_value
    end
    result, ir_front_left_center_distance = simReadProximitySensor(ir_front_left_center)
    if not ir_front_left_center_distance then
        ir_front_left_center_distance = max_ir_value
    end
    result, ir_front_right_center_distance = simReadProximitySensor(ir_front_right_center)
    if not ir_front_right_center_distance then
        ir_front_right_center_distance = max_ir_value
    end
    result, ir_front_right_distance = simReadProximitySensor(ir_front_right)
    if not ir_front_right_distance then
        ir_front_right_distance = max_ir_value
    end

    -- Make the point cloud data, x, y and z data for every point
    local point_cloud_data = {}
    -- Left IR (42.5 degrees)
    table.insert(point_cloud_data, (ir_front_left_distance + ir_offset_from_center) * math.cos(0.74))
    table.insert(point_cloud_data, (ir_front_left_distance + ir_offset_from_center) * math.sin(0.74))
    table.insert(point_cloud_data, 0.0)
    -- Left center IR (17.5 degrees)
    table.insert(point_cloud_data, (ir_front_left_center_distance + ir_offset_from_center) * math.cos(0.305))
    table.insert(point_cloud_data, (ir_front_left_center_distance + ir_offset_from_center) * math.sin(0.305))
    table.insert(point_cloud_data, 0.0)
    -- Right center IR (-17.5 degrees)
    table.insert(point_cloud_data, (ir_front_right_center_distance + ir_offset_from_center) * math.cos(-0.305))
    table.insert(point_cloud_data, (ir_front_right_center_distance + ir_offset_from_center) * math.sin(-0.305))
    table.insert(point_cloud_data, 0.0)
    -- Right IR (-42.5 degrees)
    table.insert(point_cloud_data, (ir_front_right_distance + ir_offset_from_center) * math.cos(-0.74))
    table.insert(point_cloud_data, (ir_front_right_distance + ir_offset_from_center) * math.sin(-0.74))
    table.insert(point_cloud_data, 0.0)

    -- Publish the IR sensors as a point cloud, this is useful for mapping in ROS
    -- Reference: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
    local cloud_data = {}
    cloud_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_point_cloud"}
    -- Describes the channels and their layout in the binary data blob
    local field_x = {name = "x", offset = 0, datatype = 7, count = 1}
    local field_y = {name = "y", offset = 4, datatype = 7, count = 1}
    local field_z = {name = "z", offset = 8, datatype = 7, count = 1}
    cloud_data['fields'] = {field_x, field_y, field_z}
    cloud_data['is_bigendian'] = false
    -- Only one point high
    cloud_data['height'] = 1
    cloud_data['width'] = 4
    -- Length of a point in bytes
    cloud_data['point_step'] = 12
    -- Length of a row in bytes
    cloud_data['row_step'] = cloud_data['width'] * cloud_data['point_step']
    cloud_data['data'] = simPackFloats(point_cloud_data)
    cloud_data['is_dense'] = true
    simExtRosInterface_publish(cloud_pub, cloud_data)

    -- Send the position of the point cloud sensor, which is the center of the robot, vertically offset
    -- Reference: http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/TransformStamped.html
    local cloud_transform = {}
    cloud_transform['header'] = {seq = 0, stamp = frame_stamp, frame_id = "base_link"}
    cloud_transform['child_frame_id'] = "ir_point_cloud"
    cloud_transform['transform'] = {translation={x=0, y=0, z=0.0345}, rotation={x=0, y=0, z=0, w=1.0}}
    simExtRosInterface_sendTransform(cloud_transform)

    -- Send the IR sensors are range sensors of infra-red type
    -- Reference: http://docs.ros.org/api/sensor_msgs/html/msg/Range.htmls
    local ir_front_left_data = {}
    ir_front_left_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_front_left"}
    ir_front_left_data['radiation_type'] = 1
    ir_front_left_data['field_of_view'] = 0
    ir_front_left_data['min_range'] = 0
    ir_front_left_data['max_range'] = max_ir_value
    ir_front_left_data['range'] = ir_front_left_distance
    simExtRosInterface_publish(ir_front_left_pub, ir_front_left_data)

    local ir_front_left_center_data = {}
    ir_front_left_center_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_front_left_center"}
    ir_front_left_center_data['radiation_type'] = 1
    ir_front_left_center_data['field_of_view'] = 0
    ir_front_left_center_data['min_range'] = 0
    ir_front_left_center_data['max_range'] = max_ir_value
    ir_front_left_center_data['range'] = ir_front_left_center_distance
    simExtRosInterface_publish(ir_front_left_center_pub, ir_front_left_center_data)

    local ir_front_right_data = {}
    ir_front_right_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_front_right"}
    ir_front_right_data['radiation_type'] = 1
    ir_front_right_data['field_of_view'] = 0
    ir_front_right_data['min_range'] = 0
    ir_front_right_data['max_range'] = max_ir_value
    ir_front_right_data['range'] = ir_front_right_distance
    simExtRosInterface_publish(ir_front_right_pub, ir_front_right_data)

    local ir_front_right_center_data = {}
    ir_front_right_center_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_front_right_center"}
    ir_front_right_center_data['radiation_type'] = 1
    ir_front_right_center_data['field_of_view'] = 0
    ir_front_right_center_data['min_range'] = 0
    ir_front_right_center_data['max_range'] = max_ir_value
    ir_front_right_center_data['range'] = ir_front_right_center_distance
    simExtRosInterface_publish(ir_front_right_center_pub, ir_front_right_center_data)

    -- Send the clock message to synchronise with ROS
    simExtRosInterface_publish(clock_pub, {clock = frame_stamp})
end

if (sim_call_type == sim_childscriptcall_cleanup) then
    -- Properly shut down all publishers and subscribers
    simExtRosInterface_shutdownPublisher(joints_pub)
    simExtRosInterface_shutdownPublisher(odom_pub)
    simExtRosInterface_shutdownPublisher(clock_pub)

    simExtRosInterface_shutdownPublisher(linear_camera_pub)
    simExtRosInterface_shutdownPublisher(cloud_pub)
    simExtRosInterface_shutdownPublisher(ir_front_left_pub)
    simExtRosInterface_shutdownPublisher(ir_front_left_center_pub)
    simExtRosInterface_shutdownPublisher(ir_front_right_pub)
    simExtRosInterface_shutdownPublisher(ir_front_right_center_pub)
    simExtRosInterface_shutdownPublisher(ir_under_left_pub)
    simExtRosInterface_shutdownPublisher(ir_under_right_pub)

    simExtRosInterface_shutdownSubscriber(speed_sub)
end
