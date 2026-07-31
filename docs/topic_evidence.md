# Topic evidence

Captured 2026-07-31 13:01 UTC | ROS 2 Jazzy | Isaac Sim 5.1

## ros2 topic list
```
/camera/camera_info
/camera/depth
/camera/image_raw
/clicked_point
/clock
/cmd_vel
/initialpose
/joint_states
/move_base_simple/goal
/odom
/parameter_events
/points
/rosout
/tf
/tf_static
```

## ros2 topic info
```
Type: rosgraph_msgs/msg/Clock
Publisher count: 1
Subscription count: 0
--- /clock
Type: tf2_msgs/msg/TFMessage
Publisher count: 2
Subscription count: 0
--- /tf
Type: tf2_msgs/msg/TFMessage
Publisher count: 1
Subscription count: 0
--- /tf_static
Type: sensor_msgs/msg/JointState
Publisher count: 1
Subscription count: 0
--- /joint_states
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 1
--- /odom
Type: sensor_msgs/msg/PointCloud2
Publisher count: 1
Subscription count: 0
--- /points
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 0
--- /camera/image_raw
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 0
--- /camera/depth
Type: sensor_msgs/msg/CameraInfo
Publisher count: 1
Subscription count: 0
--- /camera/camera_info
/imu : NOT PRESENT
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1
--- /cmd_vel
```

## ros2 topic hz
### /clock
```
	min: 0.019s max: 0.025s std dev: 0.00060s window: 257
average rate: 50.808
	min: 0.019s max: 0.025s std dev: 0.00061s window: 308
failed to shutdown: rcl_shutdown already called on the given context, at ./src/rcl/init.c:333
no messages within 8s
```
### /tf
```
average rate: 99.392
	min: 0.000s max: 0.023s std dev: 0.00951s window: 500
average rate: 99.344
	min: 0.000s max: 0.023s std dev: 0.00952s window: 600
no messages within 8s
```
### /tf_static
```
no messages within 8s
```
### /joint_states
```
average rate: 50.612
	min: 0.019s max: 0.022s std dev: 0.00053s window: 256
average rate: 50.592
	min: 0.019s max: 0.022s std dev: 0.00053s window: 307
no messages within 8s
```
### /odom
```
average rate: 50.895
	min: 0.019s max: 0.024s std dev: 0.00054s window: 257
average rate: 50.904
	min: 0.019s max: 0.024s std dev: 0.00055s window: 308
no messages within 8s
```
### /points
```
average rate: 45.851
	min: 0.018s max: 0.060s std dev: 0.00622s window: 186
average rate: 46.290
	min: 0.018s max: 0.060s std dev: 0.00586s window: 235
no messages within 8s
```
### /camera/image_raw
```
	min: 0.015s max: 0.086s std dev: 0.01163s window: 197
average rate: 38.449
	min: 0.015s max: 0.086s std dev: 0.01173s window: 237
failed to initialize wait set: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called., at ./src/rcl/wait.c:130
no messages within 8s
```
### /camera/depth
```
average rate: 42.555
	min: 0.017s max: 0.047s std dev: 0.00654s window: 217
average rate: 41.876
	min: 0.017s max: 0.047s std dev: 0.00711s window: 256
no messages within 8s
```
### /camera/camera_info
```
	min: 0.019s max: 0.022s std dev: 0.00055s window: 256
average rate: 50.880
	min: 0.019s max: 0.022s std dev: 0.00053s window: 308
failed to initialize wait set: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called., at ./src/rcl/wait.c:130
no messages within 8s
```
### /imu
```
WARNING: topic [/imu] does not appear to be published yet
no messages within 8s
```

## Joint names present in /joint_states
```
['front_left_wheel_joint', 'rear_left_wheel_joint', 'front_right_wheel_joint', 'rear_right_wheel_joint']
---
```

## /odom frame check (expect frame_id: odom, child_frame_id: base_link)
```
header:
  stamp:
    sec: 700
    nanosec: 16703175
  frame_id: odom
child_frame_id: base_link
pose:
  pose:
    position:
      x: 8.907759666442871
      y: 7.752635955810547
      z: -0.1558562070131302
```
