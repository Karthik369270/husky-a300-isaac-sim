# Topic evidence

Captured 2026-07-31 09:45 UTC | ROS 2 Jazzy | Isaac Sim 5.1

## ros2 topic list
```
/clock
/cmd_vel
/joint_states
/odom
/parameter_events
/rosout
/tf
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
/tf_static : NOT PRESENT
Type: sensor_msgs/msg/JointState
Publisher count: 1
Subscription count: 0
--- /joint_states
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 0
--- /odom
/points : NOT PRESENT
/camera/image_raw : NOT PRESENT
/camera/depth : NOT PRESENT
/camera/camera_info : NOT PRESENT
/imu : NOT PRESENT
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1
--- /cmd_vel
```

## ros2 topic hz
### /clock
```
average rate: 117.935
	min: 0.008s max: 0.012s std dev: 0.00047s window: 594
average rate: 117.982
	min: 0.008s max: 0.012s std dev: 0.00050s window: 713
no messages within 8s
```
### /tf
```
average rate: 222.088
	min: 0.000s max: 0.012s std dev: 0.00407s window: 892
average rate: 221.948
	min: 0.000s max: 0.013s std dev: 0.00407s window: 1114
no messages within 8s
```
### /tf_static
```
WARNING: topic [/tf_static] does not appear to be published yet
no messages within 8s
```
### /joint_states
```
average rate: 117.243
	min: 0.008s max: 0.012s std dev: 0.00048s window: 589
average rate: 117.143
	min: 0.008s max: 0.012s std dev: 0.00049s window: 706
no messages within 8s
```
### /odom
```
average rate: 119.109
	min: 0.007s max: 0.012s std dev: 0.00051s window: 598
average rate: 118.859
	min: 0.007s max: 0.012s std dev: 0.00051s window: 716
no messages within 8s
```
### /points
```
WARNING: topic [/points] does not appear to be published yet
no messages within 8s
```
### /camera/image_raw
```
WARNING: topic [/camera/image_raw] does not appear to be published yet
no messages within 8s
```
### /camera/depth
```
WARNING: topic [/camera/depth] does not appear to be published yet
no messages within 8s
```
### /camera/camera_info
```
WARNING: topic [/camera/camera_info] does not appear to be published yet
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
    sec: 1965
    nanosec: 783435856
  frame_id: odom
child_frame_id: base_link
pose:
  pose:
    position:
      x: -0.00010101046063937247
      y: 4.36342961762648e-09
      z: -0.1558551788330078
```
