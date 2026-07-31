# Topic evidence

Captured 2026-07-31 12:54 UTC | ROS 2 Jazzy | Isaac Sim 5.1

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
Subscription count: 1
--- /tf
Type: tf2_msgs/msg/TFMessage
Publisher count: 1
Subscription count: 1
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
Subscription count: 1
--- /points
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 1
--- /camera/image_raw
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 1
--- /camera/depth
Type: sensor_msgs/msg/CameraInfo
Publisher count: 1
Subscription count: 1
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
average rate: 42.229
	min: 0.022s max: 0.027s std dev: 0.00082s window: 215
average rate: 42.175
	min: 0.022s max: 0.027s std dev: 0.00084s window: 257
no messages within 8s
```
### /tf
```
average rate: 84.783
	min: 0.000s max: 0.025s std dev: 0.01112s window: 430
average rate: 84.722
	min: 0.000s max: 0.025s std dev: 0.01113s window: 516
no messages within 8s
```
### /tf_static
```
no messages within 8s
```
### /joint_states
```
average rate: 42.060
	min: 0.022s max: 0.027s std dev: 0.00075s window: 215
average rate: 42.031
	min: 0.022s max: 0.027s std dev: 0.00079s window: 257
no messages within 8s
```
### /odom
```
average rate: 42.518
	min: 0.022s max: 0.026s std dev: 0.00075s window: 216
average rate: 42.516
	min: 0.022s max: 0.026s std dev: 0.00075s window: 259
no messages within 8s
```
### /points
```
average rate: 40.168
	min: 0.019s max: 0.070s std dev: 0.00604s window: 206
average rate: 39.704
	min: 0.019s max: 0.070s std dev: 0.00642s window: 244
no messages within 8s
```
### /camera/image_raw
```
average rate: 28.923
	min: 0.018s max: 0.140s std dev: 0.02000s window: 154
average rate: 28.512
	min: 0.018s max: 0.140s std dev: 0.02022s window: 181
no messages within 8s
```
### /camera/depth
```
average rate: 34.281
	min: 0.018s max: 0.117s std dev: 0.01351s window: 175
average rate: 34.773
	min: 0.018s max: 0.117s std dev: 0.01287s window: 213
no messages within 8s
```
### /camera/camera_info
```
average rate: 42.359
	min: 0.022s max: 0.027s std dev: 0.00083s window: 130
average rate: 42.320
	min: 0.022s max: 0.027s std dev: 0.00081s window: 173
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
    sec: 369
    nanosec: 766685951
  frame_id: odom
child_frame_id: base_link
pose:
  pose:
    position:
      x: 8.907759666442871
      y: 7.752635955810547
      z: -0.1558562070131302
```
