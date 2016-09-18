function speedMessage_callback(msg)
    -- Receieve and interpret the twist message for platform control
    -- Reference: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html

    -- Values are in meters
    local wheel_radius = 0.035
    local wheel_separation = 0.1056

    -- Transform the cmd_vel into the left and right wheel speeds depending on the wheel geometry for differential drive
    local speed_left = msg['linear']['x'] / wheel_radius - msg['angular']['z'] * wheel_separation / wheel_radius
    local speed_right = msg['linear']['x'] / wheel_radius + msg['angular']['z'] * wheel_separation / wheel_radius

    -- Apply the motor speed to the motors
    simSetJointTargetVelocity(joint_drive_left, speed_left)
    simSetJointTargetVelocity(joint_drive_right, speed_right)
end

if (sim_call_type == sim_childscriptcall_initialization) then
    -- Greeting message
    simAddStatusbarMessage('Starting robot simulation')

    -- Define constants for the simulation
    -- Offset between the body and the ground plane
    body_origin_height = 0.047
    -- Offset (radius) between the center of the body and the front IR sensors
    ir_offset_from_center = 0.06176
    -- Maximum IR sensing distance
    max_ir_value = 0.2

    -- Publish the simulation time
    clock_pub = simExtRosInterface_advertise('/clock', 'rosgraph_msgs/Clock')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(clock_pub)

    -- Get camera handler
    linear_camera = simGetObjectHandle('linear_camera')

    -- Make the camera and camera information publishers
    linear_camera_pub = simExtRosInterface_advertise('/linear_camera', 'sensor_msgs/Image')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(linear_camera_pub)

    -- Get the motor handlers
    joint_drive_left = simGetObjectHandle('joint_drive_left')
    joint_drive_right = simGetObjectHandle('joint_drive_right')

    -- Make the joint state (motor position) publisher
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
    -- The LaserScan topic could also be used, but the rays are not equally spaced and it makes it akward
    -- so rather than having a non-perfect LaserScan topic a perfect PointCloud is later transformed to LaserScan
    cloud_pub = simExtRosInterface_advertise('/pointcloud', 'sensor_msgs/PointCloud2')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(cloud_pub)

    -- Get the body handler
    robot = simGetObjectHandle('robot')

    -- Make the odomety publisher
    odom_pub = simExtRosInterface_advertise('/odom', 'nav_msgs/Odometry')
    simExtRosInterface_publisherTreatUInt8ArrayAsString(odom_pub)

    -- Subscribe to messages of type cmd_vel of type Twist to set speed for the motors
    speed_sub = simExtRosInterface_subscribe('/cmd_vel', 'geometry_msgs/Twist', 'speedMessage_callback')
    simExtRosInterface_subscriberTreatUInt8ArrayAsString(speed_sub)
end

if (sim_call_type == sim_childscriptcall_sensing) then
    local frame_stamp = simGetSystemTime()

    -- Get the motor positions
    local position_left = simGetJointPosition(joint_drive_left)
    local position_right = simGetJointPosition(joint_drive_right)

    -- We do not care about the velocity in the virutal representation
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

    -- Get the robot position
    local position = simGetObjectPosition(robot, -1)
    local rotation = simGetObjectQuaternion(robot, -1)
    local velocity_linear, velocity_angular = simGetObjectVelocity(robot)

    local base_data = {}
    base_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "odom"}
    base_data['child_frame_id'] = "base_link"
    base_data['transform'] = {}
    base_data['transform']['translation'] = {x = position[1], y = position[2], z = position[3] - body_origin_height}
    base_data['transform']['rotation'] = {x = rotation[1], y = rotation[2], z = rotation[3], w = rotation[4]}
    simExtRosInterface_sendTransform(base_data)

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

    -- Publish the image of the linear camera:
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

    -- Publish the image of the camera:
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
    -- When the distance sensor does not detect anything it returns "nil", which is annoying, transform them to -1
    result, ir_front_left_distance = simReadProximitySensor(ir_front_left)
    if not ir_front_left_distance then
        ir_front_left_distance = -1
    end
    result, ir_front_left_center_distance = simReadProximitySensor(ir_front_left_center)
    if not ir_front_left_center_distance then
        ir_front_left_center_distance = -1
    end
    result, ir_front_right_center_distance = simReadProximitySensor(ir_front_right_center)
    if not ir_front_right_center_distance then
        ir_front_right_center_distance = -1
    end
    result, ir_front_right_distance = simReadProximitySensor(ir_front_right)
    if not ir_front_right_distance then
        ir_front_right_distance = -1
    end

    -- Make the point cloud data, make sure something is visible
    local point_cloud_data = {}
    -- x, y and z data for every point
    local point_counter = 0
    -- Left IR (42.5 degrees)
    if ir_front_left_distance > 0 then
        table.insert(point_cloud_data, (ir_front_left_distance + ir_offset_from_center) * math.cos(0.74))
        table.insert(point_cloud_data, (ir_front_left_distance + ir_offset_from_center) * math.sin(0.74))
        table.insert(point_cloud_data, 0.0)
        point_counter = point_counter + 1
    end
    -- Left center IR (17.5 degrees)
    if ir_front_left_center_distance > 0 then
        table.insert(point_cloud_data, (ir_front_left_center_distance + ir_offset_from_center) * math.cos(0.305))
        table.insert(point_cloud_data, (ir_front_left_center_distance + ir_offset_from_center) * math.sin(0.305))
        table.insert(point_cloud_data, 0.0)
        point_counter = point_counter + 1
    end
    -- Right center IR (-17.5 degrees)
    if ir_front_right_center_distance > 0 then
        table.insert(point_cloud_data, (ir_front_right_center_distance + ir_offset_from_center) * math.cos(-0.305))
        table.insert(point_cloud_data, (ir_front_right_center_distance + ir_offset_from_center) * math.sin(-0.305))
        table.insert(point_cloud_data, 0.0)
        point_counter = point_counter + 1
    end
    -- Right IR (-42.5 degrees)
    if ir_front_right_distance > 0 then
        table.insert(point_cloud_data, (ir_front_right_distance + ir_offset_from_center) * math.cos(-0.74))
        table.insert(point_cloud_data, (ir_front_right_distance + ir_offset_from_center) * math.sin(-0.74))
        table.insert(point_cloud_data, 0.0)
        point_counter = point_counter + 1
    end

    -- Only publish point cloud when there is something to publish
    --if point_counter > 0 then
    local cloud_data = {}
    cloud_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_point_cloud"}

    local field_x = {name = "x", offset = 0, datatype = 7, count = 1}
    local field_y = {name = "y", offset = 4, datatype = 7, count = 1}
    local field_z = {name = "z", offset = 8, datatype = 7, count = 1}

    -- Describes the channels and their layout in the binary data blob
    cloud_data['fields'] = {field_x, field_y, field_z}
    cloud_data['is_bigendian'] = false
    -- Actual point data, size is (row_step*height)
    cloud_data['height'] = 1
    cloud_data['width'] = point_counter
    -- Length of a point in bytes
    cloud_data['point_step'] = 12
    -- Length of a row in bytes
    cloud_data['row_step'] = cloud_data['width'] * cloud_data['point_step']
    cloud_data['data'] = simPackFloats(point_cloud_data)
    cloud_data['is_dense'] = true
    simExtRosInterface_publish(cloud_pub, cloud_data)
    --end

    -- Send the position of the point cloud sensor, which is the center of the robot, vertically offset
    local cloud_transform = {}
    cloud_transform['header'] = {seq = 0, stamp = frame_stamp, frame_id = "base_link"}
    cloud_transform['child_frame_id'] = "ir_point_cloud"
    cloud_transform['transform'] = {translation={x=0, y=0, z=0.0345}, rotation={x=0, y=0, z=0, w=1.0}}
    simExtRosInterface_sendTransform(cloud_transform)

    -- Reference: http://docs.ros.org/api/sensor_msgs/html/msg/Range.htmls
    local ir_front_left_data = {}
    ir_front_left_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_front_left"}
    -- Infrared type
    ir_front_left_data['radiation_type'] = 1
    ir_front_left_data['field_of_view'] = 0
    ir_front_left_data['min_range'] = 0
    ir_front_left_data['max_range'] = max_ir_value
    ir_front_left_data['range'] = ir_front_left_distance
    -- When the sensor does not detect anything it will return maximum value
    if ir_front_left_data['range'] < 0 then
        ir_front_left_data['range'] = max_ir_value
    end
    simExtRosInterface_publish(ir_front_left_pub, ir_front_left_data)

    local ir_front_left_center_data = {}
    ir_front_left_center_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_front_left_center"}
    -- Infrared type
    ir_front_left_center_data['radiation_type'] = 1
    ir_front_left_center_data['field_of_view'] = 0
    ir_front_left_center_data['min_range'] = 0
    ir_front_left_center_data['max_range'] = max_ir_value
    ir_front_left_center_data['range'] = ir_front_left_center_distance
    -- When the sensor does not detect anything it will return maximum value
    if ir_front_left_center_data['range'] < 0 then
        ir_front_left_center_data['range'] = max_ir_value
    end
    simExtRosInterface_publish(ir_front_left_center_pub, ir_front_left_center_data)

    local ir_front_right_data = {}
    ir_front_right_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_front_right"}
    -- Infrared type
    ir_front_right_data['radiation_type'] = 1
    ir_front_right_data['field_of_view'] = 0
    ir_front_right_data['min_range'] = 0
    ir_front_right_data['max_range'] = max_ir_value
    ir_front_right_data['range'] = ir_front_right_distance
    -- When the sensor does not detect anything it will return maximum value
    if ir_front_right_data['range'] < 0 then
        ir_front_right_data['range'] = max_ir_value
    end
    simExtRosInterface_publish(ir_front_right_pub, ir_front_right_data)

    local ir_front_right_center_data = {}
    ir_front_right_center_data['header'] = {seq = 0, stamp = frame_stamp, frame_id = "ir_front_right_center"}
    -- Infrared type
    ir_front_right_center_data['radiation_type'] = 1
    ir_front_right_center_data['field_of_view'] = 0
    ir_front_right_center_data['min_range'] = 0
    ir_front_right_center_data['max_range'] = max_ir_value
    ir_front_right_center_data['range'] = ir_front_right_center_distance
    -- When the sensor does not detect anything it will return maximum value
    if ir_front_right_center_data['range'] < 0 then
        ir_front_right_center_data['range'] = max_ir_value
    end
    simExtRosInterface_publish(ir_front_right_center_pub, ir_front_right_center_data)

    -- Send the clock message to synchronise with ROS
    simExtRosInterface_publish(clock_pub, {clock = frame_stamp})
end

if (sim_call_type == sim_childscriptcall_cleanup) then
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
