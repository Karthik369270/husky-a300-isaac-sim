# Design choices, limitations and challenges

*Deliverable 4 of the submission. Fill in as work proceeds - do not leave to the end.*

## Approach

TBD - one paragraph: URDF audited offline first, then imported via the Isaac Sim
Python API, articulation configured in code, ROS 2 bridge action graphs built
with OmniGraph from Python, everything containerised.

## Design choices

### Why `merge_fixed_joints=False`
TBD - see docs/urdf_audit.md

### Camera left facing rearward
TBD

### Optical frame rotation on the USD camera prim
TBD

### Drive type and gains
TBD - velocity drives on the four continuous joints, stiffness/damping values used
and how they were arrived at.

### Odometry source
TBD - how /odom is produced and why frame_id/child_frame_id are set as they are.

### Two-container split (Isaac Sim + ROS 2 Jazzy)
TBD - Isaac Sim 5.1 ships internal ROS 2 Jazzy libraries but not the ROS 2 CLI,
RViz or teleop tooling. Rather than installing a second ROS distribution into the
Isaac image, a `ros:jazzy` sidecar shares the host network. Keeps the Isaac image
unmodified and the build recipe short.

## Limitations

TBD - be specific and honest. Anything not implemented, anything approximated,
anything that only works under certain conditions.

## Challenges

TBD

## What I would do with more time

TBD
