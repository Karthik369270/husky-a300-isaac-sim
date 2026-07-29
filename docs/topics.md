# Expected topics and frames

Publish these names exactly, so every submission can be tested the same way. Set
`use_sim_time` to true on every node.

## Required

| Topic | Type | Notes |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time. Everything else depends on it |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Fixed transforms |
| `/joint_states` | `sensor_msgs/msg/JointState` | Must contain the four wheel joints |
| `/odom` | `nav_msgs/msg/Odometry` | `frame_id: odom`, `child_frame_id: base_link` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Subscribed, not published |

The four driven joints, named exactly as in the URDF:

```
front_left_wheel_joint    rear_left_wheel_joint
front_right_wheel_joint   rear_right_wheel_joint
```

## Bonus

| Topic | Type | Frame |
|---|---|---|
| `/points` | `sensor_msgs/msg/PointCloud2` | `lidar3d_0_sensor_link` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `camera_0_left_camera_frame_optical` |
| `/camera/depth` | `sensor_msgs/msg/Image` | `camera_0_left_camera_frame_optical` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_0_left_camera_frame_optical` |
| `/imu` | `sensor_msgs/msg/Imu` | `imu_0_link` |

## Two things about the sensor frames

**Importing the URDF does not give you working sensors.** `lidar3d_0_sensor_link` and
`camera_0_left_camera_frame_optical` are empty coordinate frames. The sensors themselves
have to be created in the simulator at those frames and wired up.

**The camera points to the rear of the robot.** The sensor arch sits behind the chassis
and the ZED is mounted facing backwards, so it looks out over the cargo bed rather than
over the front. That is how the robot is specified. Note it, do not silently correct it.

Image topics are published in the optical frame by convention, not the camera body
frame. A ROS optical frame is Z forward, X right, Y down, which is not the same
convention most simulators use for their camera prims.

## Frame tree

```
odom
└── base_link
    ├── chassis_link
    │   ├── base_footprint
    │   ├── front_bumper_link
    │   ├── left_suspension_beam_spacer_link
    │   │   └── left_suspension_beam_link
    │   │       ├── front_left_motor_link
    │   │       │   └── front_left_wheel_link (continuous)
    │   │       └── rear_left_motor_link
    │   │           └── rear_left_wheel_link (continuous)
    │   ├── rear_bumper_link
    │   └── right_suspension_beam_spacer_link
    │       └── right_suspension_beam_link
    │           ├── front_right_motor_link
    │           │   └── front_right_wheel_link (continuous)
    │           └── rear_right_motor_link
    │               └── rear_right_wheel_link (continuous)
    ├── enclosure_enclosure_link
    │   ├── enclosure_access_panels_link
    │   └── sensor_arch_link
    │       ├── camera_0_camera_link
    │       │   └── camera_0_camera_center
    │       │       ├── camera_0_baro_link
    │       │       ├── camera_0_left_camera_frame
    │       │       │   ├── camera_0_left_camera_frame_optical
    │       │       │   └── camera_0_temp_left_link
    │       │       ├── camera_0_mag_link
    │       │       └── camera_0_right_camera_frame
    │       │           ├── camera_0_right_camera_frame_optical
    │       │           └── camera_0_temp_right_link
    │       └── lidar3d_0_base_link
    │           └── lidar3d_0_link
    │               ├── lidar3d_0_cap_link
    │               └── lidar3d_0_sensor_link
    └── imu_0_link
```

Note that the wheels are not direct children of `base_link`. They hang off the
suspension and motor chain, so `base_link` to a wheel is four transforms, not one.

## Checking your work

```
ros2 topic list
ros2 topic hz /joint_states
ros2 run tf2_tools view_frames
```

Paste the output of `ros2 topic list` and `ros2 topic hz` for each topic into your
README.
