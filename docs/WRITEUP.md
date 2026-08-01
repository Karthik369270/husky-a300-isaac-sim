# Design choices, limitations and challenges

## Approach

The description pack was audited offline before any simulator was available.
That was initially forced by circumstance — the only local machine is a 6 GB
RTX 4050 laptop, well under Isaac Sim's requirements — but it turned out to be
the right order of work regardless. Parsing the URDF, resolving the frame tree
by hand and measuring the mesh geometry directly meant that when the simulator
did become available, the failures that appeared were recognisable rather than
mysterious.

Isaac Sim was run on a rented RTX 4090. Everything is constructed from the
Python API: `build_scene.py` composes the stage and articulation,
`ros_graphs.py` builds the ROS 2 OmniGraph and the skid-steer controller, and
`sensors.py` creates the camera and LiDAR at the frames the URDF provides.

---

## Design choices

### `merge_fixed_joints = False`

The single most consequential setting in the import config. Twelve links in the
URDF have no inertial block and exist only as coordinate frames, including every
sensor mount. The importer's default behaviour merges fixed-joint children into
their parent rigid body — which would have silently deleted
`lidar3d_0_sensor_link`, `camera_0_left_camera_frame_optical` and `imu_0_link`
from the stage, leaving the sensors nowhere to attach and a published `/tf` tree
that no longer matched `topics.md`.

This was identified from the offline audit rather than discovered in the
simulator. The import log confirms it: warnings like `No mass specified for link
lidar3d_0_sensor_link` are the massless frames surviving as intended.

### Referencing the warehouse rather than opening it

The first working implementation called `open_stage()` on the warehouse USD
directly. It failed with `Cannot save layer ... saving not allowed`, followed by
`attempted member lookup on NULL TfRefPtr<UsdStage>`.

The cause is USD layer composition. Opening the environment makes NVIDIA's
read-only S3 layer the edit target; the URDF importer then attempts to author
the robot into that layer, is refused by the resolver, and the stage is
invalidated.

The fix is architectural rather than a workaround: create a local anonymous
stage, define `/World` as the default prim, and add the warehouse as a
*reference* at `/World/Warehouse`. The environment stays read-only and composed,
while all authoring — robot, physics scene, sensors, graphs — happens on the
local root layer.

### Discovering prims by traversal, not by path

The URDF robot name is `a300-00000`. USD prim names cannot contain a hyphen, so
the importer sanitises it to `a300_00000` — a transformation not reliably
predictable from the URDF alone. Rather than hardcode a guess, `build_scene.py`
traverses the stage matching link and joint names, and dumps every joint-like
prim if a lookup fails. A wrong assumption costs one run instead of an hour.

### Velocity drives, stiffness zero

All four wheel joints use `UsdPhysics.DriveAPI` angular drives with stiffness 0,
damping 1e4 and max force 1e5. Stiffness zero is what makes these velocity
drives rather than position drives. Target velocity is written in **degrees per
second**, which is the unit the API expects — a units mismatch here produces a
robot that either sits still or launches itself.

### Skid-steer kinematics in Python rather than OmniGraph

`isaacsim.robot.wheeled_robots.DifferentialController` emits a two-element
velocity command, left and right. The Husky has four driven joints. Expanding
two to four inside OmniGraph requires array-construction nodes whose exact
behaviour could not be verified without burning rented GPU time on trial and
error.

The kinematics therefore live in `SkidSteerController`: read the latest Twist
from the `ROS2SubscribeTwist` node's output attributes each step, compute left
and right wheel velocities from the effective track, and write the four drive
targets. This satisfies the "Python API only, no GUI" constraint fully, and has
the practical advantage that the scrub multiplier becomes a runtime parameter
that the measurement experiment can sweep.

The publisher side — clock, TF, joint states, odometry, and the `/cmd_vel`
subscriber itself — is all genuine OmniGraph, built with `og.Controller.edit`.

### Two transform tree publishers

`ROS2PublishTransformTree` publishes the articulation's internal tree to `/tf`.
A second instance with `staticPublisher` set publishes to `/tf_static`.
`ROS2PublishRawTransformTree` publishes `odom -> base_link` separately, driven by
the same `IsaacComputeOdometry` node that feeds `/odom`, so the transform and
the odometry message can never disagree.

This is why `/tf` reports two publishers — by design, not duplication.

### Simulation time everywhere

Every publisher's `timeStamp` input is wired to `IsaacReadSimulationTime`, and
`/clock` is published from that same source. Nothing derives a timestamp from
wall time.

### Foxglove rather than RViz

The assignment permits either. The instance is headless with no display, so RViz
would have required X forwarding over the internet. Foxglove Bridge exposes a
websocket that tunnels cleanly over SSH with no open ports, needs no X server,
and renders in a browser on the local machine. An RViz configuration is included
in `config/husky.rviz` for completeness.

---

## Challenges

Roughly two thirds of the elapsed time went to environment problems rather than
to the assignment itself. Recording them since the brief asks for challenges
encountered.

**NGC registry authentication.** The `nvcr.io/nvidia/isaac-sim:5.1.0` container
could not be pulled — repeated `docker login failed` with an empty
`unauthorized:` reason, across three freshly generated NGC personal keys with
full service scoping. Rather than continue debugging a credential problem,
Isaac Sim was installed from the public PyPI distribution onto a plain
`nvidia/cuda:12.8.0-devel-ubuntu24.04` image, which requires no registry
authentication at all. Same 5.1.0 release, no auth surface.

**Driver version, in the wrong direction.** The first machine ran driver
595.58.03 / CUDA 13.2 — *newer* than Isaac Sim 5.1 supports. It crashed on
startup inside `librtx.scenedb.plugin.so` at `carbOnPluginStartup`, on every
experience config including the minimal ones. Vulkan enumerated the 4090
correctly and 24 GB of VRAM was free, which ruled out the obvious causes.
Re-provisioning on 580.159.03 / CUDA 13.0 resolved it immediately. The lesson is
that "580 or higher" is wrong; the RTX renderer wants the version NVIDIA
actually tested.

**Missing graphics libraries.** The CUDA base image ships no OpenGL or Vulkan
runtime, so the MDL material system failed to load `libneuray.so`. Installing
`libgl1`, `libegl1`, `libvulkan1` and the X11 support libraries fixed it — but
installing `mesa-vulkan-drivers` alongside them made things worse, registering a
software rasteriser that shadowed the NVIDIA ICD.

**OmniGraph node path resolution.** Adding the camera nodes to the existing
graph failed with `Failed to connect OnTick.outputs:tick ->
/ROS2Graph/CreateRP.inputs:execIn`. Within a single `og.Controller.edit` call,
nodes created *in that call* resolve by short name, but nodes created in an
earlier call do not — they need fully-qualified paths. The error message is
legible once you notice one side is prefixed and the other is not.

**Render product attachment.** The LiDAR published nothing while logging
`Render product not attached to RTX Lidar` every frame. The cause was that the
prim path passed to the render product had been *constructed* from the command
arguments rather than read back from the command's return value, and the actual
path differed. Using the returned prim fixed it.

Every graph-building function in `ros_graphs.py` and `sensors.py` catches
exceptions and dumps every attribute on the failing node with its resolved type.
That diagnostic turned each of these from a guessing game into a single
iteration, which mattered given the work was on metered hardware.

---

## Limitations

Enumerated in the README under *Limitations*. The ones worth emphasising:

The **container recipe was never executed** — the development environment is
itself a container without nested Docker support. It is provided as written and
labelled as untested rather than claimed as working.

The **camera image is dark** because the optical frame sits against the sensor
arch enclosure. The topic, rate, frame and calibration are all correct; what the
camera sees is largely the robot's own geometry.

The **IMU bonus is not implemented.** Isaac Sim 5.1 registers no
`ROS2PublishImu` node, so it requires an IMU sensor wired to the generic
`ROS2Publisher` with an explicit message type. Given a finite budget, an honest
and complete write-up was judged more valuable than two additional points.

---

## What I would do with more time

**Fix the contact model rather than the kinematics.** The measured scrub
multiplier of ~7 against Clearpath's hardware value of 1.75 says the simulated
tyre-ground contact is wrong, and correcting it with a kinematic fudge factor
treats the symptom. A sweep over PhysX friction, contact offset, rest offset and
solver iteration count — validated against the real platform's turning
behaviour — would be the proper fix.

**Close the loop on yaw rate.** An open-loop multiplier cannot handle a
correction that varies with commanded rate. A yaw-rate controller driven by IMU
or odometry feedback handles rate-dependence naturally and is closer to what a
real skid-steer platform does.

**Characterise the low-rate stall.** The multiplier climbing to 13.6 at
0.3 rad/s suggests a breakaway threshold below which commanded rotation produces
almost no motion. Finding that threshold explicitly would be useful to anyone
planning motion on this platform.

**Complete the sensor suite** — IMU, and the right stereo camera for a full ZED
pair, with a proper distortion model in `camera_info`.

**Validate the container** on a host with Docker available, and add a CI job that
builds it and asserts the required topics appear.
