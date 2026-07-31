# Husky A300 in NVIDIA Isaac Sim 5.1

Clearpath Husky A300 built from URDF into Isaac Sim 5.1, placed in NVIDIA's
warehouse environment, teleoperated with skid-steer differential drive, and
publishing state and sensor data to ROS 2 Jazzy.

Everything — the scene, the robot, the articulation, the drives and every ROS 2
action graph — is constructed from the **Isaac Sim Python API**. No part of this
was authored through the GUI.

---

## Demo

**Video:** https://drive.google.com/drive/folders/1ejChhwNRVRFexPjLbMvPaOqZl_3bpcQR?usp=sharing

The recording shows, in Foxglove: the full 37-frame transform tree updating
live, the LiDAR point cloud of the warehouse interior, the camera topic, and
`/odom` position changing as the robot is driven forward, in reverse and
rotating under keyboard teleoperation.

Screenshots are in [`docs/screenshots/`](docs/screenshots/).

---

## Versions

| Component | Version |
|---|---|
| Isaac Sim | 5.1.0 (installed from PyPI, see *Install method* below) |
| ROS 2 | Jazzy (`ros-jazzy-ros-base`, Ubuntu packages) |
| OS | Ubuntu 24.04 (Noble) |
| NVIDIA driver | 580.159.03 |
| CUDA | 13.0 |
| GPU | RTX 4090, 24 GB (rented, Vast.ai) |
| Python | 3.11 (Isaac Sim venv) / 3.12 (system, ROS 2) |

### Install method

Isaac Sim was installed via the public PyPI distribution rather than the
`nvcr.io/nvidia/isaac-sim:5.1.0` container, because NGC registry authentication
failed repeatedly on the cloud provider. The pip distribution is the same
5.1.0 release and requires no registry credentials:

```bash
python3.11 -m venv /isaac-env && source /isaac-env/bin/activate
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

**Driver version matters more than the documentation implies.** Isaac Sim 5.1
crashed on startup inside `rtx.scenedb` on a machine running driver 595.58.03
(CUDA 13.2) — too new. It runs correctly on 580.159.03 (CUDA 13.0), the version
NVIDIA lists. When provisioning, filter for CUDA 13.0 specifically, not "13.0 or
higher".

---

## Quick start

```bash
git clone <this repo> /workspace && cd /workspace

# verify the mesh pack before doing anything else
python3 scripts/check_meshes.py

# scene only, no ROS - does the robot import and stand up?
python scripts/build_scene.py --headless 1 --no-ros --frames 300

# full run with both sensors
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0
python scripts/build_scene.py --headless 1 --camera --lidar
```

In a second shell:

```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0
ros2 topic list
bash scripts/verify_topics.sh                       # captures docs/topic_evidence.md
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Foxglove: connect to `ws://localhost:8765`, 3D panel with fixed and display
frame `odom`, follow mode `None`, `/points` enabled, Image panel on
`/camera/image_raw`.

Containerised entry point:

```bash
docker compose -f docker/docker-compose.yml up
```

See *What does not work* regarding the container.

### Useful flags

| Flag | Purpose |
|---|---|
| `--no-ros` | Scene and articulation only, no ROS graphs |
| `--spin-test` | Open-loop wheel spin, to verify drives independently of ROS |
| `--camera` | Publish ZED RGB, depth and camera_info |
| `--lidar` | Publish the LiDAR point cloud |
| `--scrub-multiplier` | Skid-steer `wheel_separation_multiplier` (default 1.75) |
| `--wheel-friction` | Static/dynamic friction on wheel colliders |
| `--frames N` | Run N frames then exit |
| `--save-usd PATH` | Export the composed stage |

---

## Repository layout

```
scripts/build_scene.py     URDF import, articulation, drives, warehouse, main loop
scripts/ros_graphs.py      ROS 2 OmniGraph publishers + skid-steer controller
scripts/sensors.py         ZED camera and RTX LiDAR graphs
scripts/check_meshes.py    Pre-flight mesh verification
scripts/urdf_audit.py      Offline structure and mass audit
scripts/urdf_frames.py     Frame transform extraction
scripts/verify_topics.sh   Captures topic list / hz evidence
scripts/measure_scrub.py   Skid-steer scrub experiment
config/husky.rviz          RViz layout
docker/                    Compose recipe and entrypoint
resources/                 husky_a300.urdf, meshes/, LICENSE
docs/                      Audit, topic evidence, scrub measurement, write-up
```

---

## What works / what does not

| Task | Points | Status |
|---|---|---|
| URDF to USD, articulation, joints, drives, masses, colliders | 6 | **Working** |
| NVIDIA warehouse environment, robot placed in it | 2 | **Working** |
| Differential drive control and keyboard teleoperation | 4 | **Working** |
| ROS 2 bridge: `/clock` `/tf` `/tf_static` `/joint_states` `/odom`, `/cmd_vel` subscribed | 4 | **Working** |
| Visualisation of robot and transforms (Foxglove) | 2 | **Working** |
| Containerised setup with single-command run | 2 | **Provided, not executed** |
| Bonus: LiDAR point cloud to ROS 2, viewed in Foxglove | 3 | **Working**, generic profile |
| Bonus: camera image and depth to ROS 2, viewed in Foxglove | 3 | **Working**, image is dark |
| Bonus: IMU to a ROS 2 topic | 2 | **Investigated, not implemented** |

**26 of 28 points implemented.** Detail on every caveat in *Limitations* —
nothing below is claimed to work that has not been observed working.

---

## Topic evidence

Full capture in [`docs/topic_evidence.md`](docs/topic_evidence.md), produced by
`scripts/verify_topics.sh`.

### `ros2 topic list`

```
/camera/camera_info
/camera/depth
/camera/image_raw
/clock
/cmd_vel
/joint_states
/odom
/parameter_events
/points
/rosout
/tf
/tf_static
```

### `ros2 topic hz`

| Topic | Type | Rate |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | 117.9 Hz |
| `/tf` | `tf2_msgs/TFMessage` | 222.1 Hz |
| `/tf_static` | `tf2_msgs/TFMessage` | static, latched |
| `/joint_states` | `sensor_msgs/JointState` | 117.2 Hz |
| `/odom` | `nav_msgs/Odometry` | 119.1 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | ~50 Hz |
| `/points` | `sensor_msgs/PointCloud2` | ~55 Hz |
| `/cmd_vel` | `geometry_msgs/Twist` | subscribed, 0 publishers |

`/tf` carries two publishers by design: the articulation tree, and the
`odom -> base_link` transform from `ROS2PublishRawTransformTree`.

### Joint names in `/joint_states`

```
['front_left_wheel_joint', 'rear_left_wheel_joint',
 'front_right_wheel_joint', 'rear_right_wheel_joint']
```

### `/odom` frames

```
header.frame_id: "odom"
child_frame_id:  "base_link"
```

All nodes run with `use_sim_time` semantics: every timestamp is sourced from
`IsaacReadSimulationTime` and `/clock` is published from the same source.

---

## Findings from the description pack

Full analysis in [`docs/urdf_audit.md`](docs/urdf_audit.md). Everything below was
established offline, before the simulator was available, and shaped the
implementation.

### `merge_fixed_joints` must be False

Twelve links carry no `<inertial>` block and exist purely as coordinate frames —
including `lidar3d_0_sensor_link`, `camera_0_left_camera_frame_optical` and
`imu_0_link`. The importer's default merges fixed-joint children into their
parent rigid body, which would delete every sensor mount point from the stage
and produce a `/tf` tree that does not match `topics.md`.

Confirmed at import time by warnings such as
`No mass specified for link lidar3d_0_sensor_link` — those links survived
precisely because merging was disabled.

### The five millimetre-scaled meshes

Rather than trusting the `scale` attribute, raw bounding boxes were measured
from the binary STL data:

| Mesh | Triangles | Raw extents |
|---|---|---|
| `chassis_collision.stl` | 196 | 0.899 x 0.461 x 0.230 |
| `outdoor_left.stl` | 95626 | 0.336 x 0.120 x 0.336 |
| **`observer_access_panels.stl`** | 56 | **858.9 x 459.9 x 179.5** |
| **`observer_arch.stl`** | 116 | **175.7 x 650.0 x 445.4** |
| **`observer_enclosure.stl`** | 140 | **971.4 x 452.9 x 186.7** |

Three orders of magnitude apart, and exactly the three carrying
`scale="0.001 0.001 0.001"`. Note that two of the five scaled *entries* are
collision geometry, so a visual-only check would miss them. Cross-check:
`outdoor_left.stl` measures 0.336 m across against a stated wheel radius of
0.1651 m — correct as authored.

`scripts/check_meshes.py` reproduces this in one second and fails loudly if the
`meshes/` tree has been flattened.

### The camera faces the rear, and was left that way

`topics.md` asks that this be noted, not corrected. Confirmed numerically: the
local +X axis of `camera_0_left_camera_frame` points along
`[-0.9997, -0.0102, -0.0230]` in `base_link` — directly rearward with a slight
downward tilt. **No correction was applied.**

### Optical frame convention

`camera_0_left_camera_frame_optical` is textbook ROS: +Z along the view
direction, +X camera-right, +Y camera-down. A USD camera prim uses -Z forward
and +Y up, so the camera prim carries a fixed 180 degree rotation about X
relative to the optical frame. Image topics are published in the optical frame,
not the camera body frame.

### Track geometry

Resolved through the full chain (wheels are four transforms from `base_link`,
not one):

- measured track **0.5468 m** — note this differs from the 0.562 m quoted in
  RESOURCES.md; the URDF geometry is taken as authoritative
- measured wheelbase 0.5120 m
- the track is **not symmetric** about the centreline: +0.2829 left, -0.2639
  right, a 19 mm offset
- all four driven joints rotate about `axis = 0 1 0`

### Robot name sanitisation

The URDF robot name is `a300-00000`. USD prim names cannot contain a hyphen, so
the importer produces `/World/a300_00000`. The joint and link lookups in
`build_scene.py` traverse the stage by name rather than assuming a path, because
the sanitised result is not predictable from the URDF alone.

---

## Skid-steer scrub: measured, not assumed

Full method and data in [`docs/scrub_measurement.md`](docs/scrub_measurement.md).

RESOURCES.md gives Clearpath's `wheel_separation_multiplier: 1.75` and adds that
"whether that value holds in your simulator is worth measuring rather than
assuming." It does not hold.

**The drives are not the problem.** While commanding a turn, `/joint_states`
reports `[-1.452, -1.504, 1.498, 1.483]` rad/s — all four tracking their targets.
Ideal differential kinematics predicts ~0.89 rad/s of yaw. Measured: ~0.12 rad/s.
The robot achieves about **13% of the rotation its wheel speeds imply**. The
tyres are sliding.

Measured multiplier with PhysX default friction and no compensation:

| commanded (rad/s) | actual (rad/s) | multiplier |
|---|---|---|
| 0.30 | 0.042 | 7.060 |
| 0.60 | 0.089 | 6.750 |
| 0.90 | 0.116 | 7.744 |
| 1.20 | 0.171 | 7.025 |

**mean 7.145** — roughly four times Clearpath's hardware value.

Raising wheel friction to mu = 0.9 does not simply fix it, it redistributes the
error: tracking improves at speed (7.03 -> 3.67 at 1.2 rad/s) and degrades at low
rates (7.06 -> 13.59 at 0.3 rad/s). That is the signature of stiction — more grip
raises the force needed to break the tyres into slip.

**The most important result: a single scalar multiplier cannot model this.** The
required correction varies from 13.59 to 3.67 within one session, monotonically
with commanded yaw rate. `wheel_separation_multiplier` is a constant; the
phenomenon it represents is not.

The controller exposes the multiplier as `--scrub-multiplier`, defaulting to
1.75 for fidelity to the hardware configuration. The measurement is reported
rather than silently baked in, because no constant is correct.

---

## Limitations

**Container not executed end to end.** `docker/docker-compose.yml` and its
entrypoint are provided, but development ran on a cloud instance that is itself
a container and does not support nested Docker. The recipe reflects the working
environment but has not been run as written. Stated plainly rather than claimed.

**LiDAR uses a generic rotary profile.** This build ships no Ouster OS1 config —
`omni.sensors.nv.common/data/lidar/` contains Hesai, Velodyne, Luminar and
`Example_*` profiles but no OS1. `Example_Rotary` is used instead. The mount pose
and `frame_id` remain those of the OS1 as specified; the beam pattern does not
match a real OS1-128.

**LiDAR motion distortion is disabled.** `MotionBVH for lidar model not enabled`
— returns are instantaneous rather than smeared across the sweep. Real scanning
LiDAR exhibits motion distortion while the platform moves.

**The camera image is dark.** The topic publishes correctly at ~50 Hz with valid
`camera_info`, but the ZED optical frame sits directly against the sensor arch
enclosure, so the camera renders largely the inside of the robot's own geometry.
A small offset was applied along the view axis; a full fix would require either
moving the camera prim further from the mount or adding scene lighting behind
the robot. The LiDAR point cloud carries the visual demonstration instead.

**Camera distortion model.** `Unsupported physical distortion model 'None'.
Using plumb_bob with default coefficients` — `camera_info` publishes with
default distortion rather than a calibrated ZED model.

**IMU: investigated, not implemented.** Unlike the camera and LiDAR, the IMU has
no dedicated bridge node in Isaac Sim 5.1. Enumerating the registered node types
in the 5.1.0 source tree:

```
grep -rhoE '"isaacsim\.ros2\.bridge\.[A-Za-z0-9_]+"' source --include=*.py | sort -u
```

returns `ROS2PublishImage`, `ROS2PublishLaserScan`, `ROS2PublishPointCloud`,
`ROS2PublishOdometry`, `ROS2PublishJointState`, `ROS2PublishTransformTree`,
`ROS2PublishRawTransformTree`, `ROS2CameraHelper`, `ROS2CameraInfoHelper` and
`ROS2RtxLidarHelper` — but **no `ROS2PublishImu`**.

Publishing `sensor_msgs/Imu` therefore requires a different pattern from the two
bonuses that were completed: an IMU sensor created via `isaacsim.sensors.physics`
at `imu_0_link`, read through `IsaacReadIMU`, and marshalled into the generic
`isaacsim.ros2.bridge.ROS2Publisher` node with an explicit message package and
type — rather than the render-product-plus-helper pattern that the camera and
LiDAR share.

The mount frame is already resolved and available: `imu_0_link` sits at
`[0.05900, 0.00000, 0.16128]` in `base_link` with identity rotation (see
`docs/urdf_audit.md`), so placement is not the obstacle. The work is the message
marshalling through the generic publisher, which is a materially different and
less well-documented path than the one already proven twice in this repository.

With a fixed budget on metered hardware, this was scoped out in favour of
completing the documentation, since three of the four submission deliverables are
written. Given more time it is the first thing I would add — the sensor placement
is solved and only the publisher wiring remains.

**Python interpreter split.** Isaac Sim 5.1 requires Python 3.11; Ubuntu 24.04's
ROS 2 Jazzy builds `rclpy` against 3.12. Isaac Sim therefore logs
`Could not import rclpy` at startup. This is harmless — the ROS 2 bridge is C++
and unaffected, which is why all topics publish normally — but simulation code
and ROS client scripts run under separate interpreters and communicate only over
DDS. `scripts/measure_scrub.py` must be run outside the Isaac Sim venv.

**Transient collision-mesh warnings.** Occasional
`getAttributeCount called on non-existent path .../collisions/mesh_0` warnings
appear during import, naming a different link on each run. The non-determinism
suggests a Fabric race during stage composition rather than a genuinely missing
mesh. No effect on physics behaviour was observed.

---

## Licence

Robot description and meshes derive from Clearpath Robotics, BSD-3-Clause. See
`resources/LICENSE`.
