# Husky A300 in Isaac Sim 5.1 - Kemabots Robotics Simulation Assignment

Clearpath Husky A300 built from URDF into NVIDIA Isaac Sim 5.1, placed in NVIDIA's
warehouse environment, teleoperated with differential drive, publishing to ROS 2 Jazzy.

Everything is built from the **Isaac Sim Python API**. No part of the scene, the
robot, or the ROS 2 action graphs is constructed through the GUI.

---

## TODO before submitting
- [ ] Fill in every `TBD` below
- [ ] Paste `docs/topic_evidence.md` output into the Topic evidence section
- [ ] Record demo video, link it here
- [ ] Complete "What works / what does not"
- [ ] Delete this TODO block

---

## Versions

| Component | Version |
|---|---|
| Isaac Sim | 5.1.0 (`nvcr.io/nvidia/isaac-sim:5.1.0`) |
| ROS 2 | Jazzy |
| Ubuntu | 24.04 |
| NVIDIA driver | TBD |
| GPU used | TBD |

## Quick start

One command:

```bash
docker compose -f docker/docker-compose.yml up
```

This starts Isaac Sim, builds the scene from `scripts/build_scene.py`, and brings
up a ROS 2 Jazzy container on the same host network.

Teleoperate:
```bash
docker compose -f docker/docker-compose.yml exec ros bash -lc \
  "source /opt/ros/jazzy/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```

Visualise:
```bash
docker compose -f docker/docker-compose.yml exec ros bash -lc \
  "source /opt/ros/jazzy/setup.bash && rviz2 -d /workspace/config/husky.rviz"
```

Verify topics:
```bash
docker compose -f docker/docker-compose.yml exec ros /workspace/scripts/verify_topics.sh
```

## Repository layout

```
scripts/     build_scene.py, ROS 2 graph setup, teleop, measurement tools
config/      RViz configuration
docker/      compose file and entrypoint
resources/   husky_a300.urdf, meshes, LICENSE
docs/        URDF audit, topic evidence, design write-up
```

## Design notes

Full analysis in [`docs/urdf_audit.md`](docs/urdf_audit.md). The three findings
that shaped the implementation:

1. **`merge_fixed_joints=False` is mandatory.** 12 links carry no inertial block
   and exist purely as coordinate frames, including all sensor mount frames.
   Merging fixed joints would delete them from the stage.
2. **The five millimetre-scaled meshes** are all in the enclosure/arch assembly,
   and two of them are collision geometry rather than visual.
3. **The camera faces rearward.** Confirmed from the transform chain, and left
   exactly as specified rather than corrected.

## Topic evidence

TBD - paste `docs/topic_evidence.md`

## Skid-steer scrub measurement

TBD - see `docs/scrub_measurement.md`

## What works / what does not

| Task | Points | Status |
|---|---|---|
| URDF to USD, articulation, joints, drives, masses, colliders | 6 | TBD |
| Warehouse environment | 2 | TBD |
| Differential drive teleoperation | 4 | TBD |
| ROS 2 bridge: /clock /tf /joint_states /odom, /cmd_vel sub | 4 | TBD |
| RViz visualisation | 2 | TBD |
| Containerised setup | 2 | TBD |
| Bonus: LiDAR point cloud | 3 | TBD |
| Bonus: camera image + depth | 3 | TBD |
| Bonus: IMU | 2 | TBD |

## Limitations and challenges

TBD

## Licence

Robot description and meshes derive from Clearpath Robotics, BSD-3-Clause. See
`resources/LICENSE`.
