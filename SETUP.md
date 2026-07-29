# One-time setup

1. Extract this archive into `~/husky_a300_isaac`
2. Copy the `meshes/` tree from the Kemabots resource pack into `resources/`,
   preserving subdirectories (`clearpath_platform/`, `clearpath_sensors/`, `zed/`)
3. `chmod +x scripts/*.py scripts/*.sh docker/entrypoint.sh`
4. `python3 scripts/check_meshes.py`   -> must report all 18 meshes present
5. `python3 scripts/urdf_audit.py`     -> must report 35 links, 108.268 kg
6. `git init && git add -A && git commit -m "Initial scaffold"`

Then the GPU work begins: see README.md.

## File map

| Path | Purpose | Status |
|---|---|---|
| `scripts/build_scene.py` | URDF import, articulation, drives, warehouse | written, untested |
| `scripts/ros_graphs.py` | OmniGraph ROS 2 publishers + skid steer | written, untested |
| `scripts/check_meshes.py` | Pre-flight mesh verification | verified working |
| `scripts/urdf_audit.py` | Link/joint/mass audit | verified working |
| `scripts/urdf_frames.py` | Frame transform extraction (needs numpy) | verified working |
| `scripts/verify_topics.sh` | Captures topic list + hz evidence | needs a running sim |
| `scripts/measure_scrub.py` | Skid-steer multiplier experiment | needs a running sim |
| `config/husky.rviz` | RViz layout | starting point, re-save from GUI if needed |
| `docker/docker-compose.yml` | Two-service run recipe | untested |
| `docker/entrypoint.sh` | Isaac Sim launch | untested |
| `docs/urdf_audit.md` | Offline analysis writeup | submission-ready |
| `docs/WRITEUP.md` | Design choices | skeleton, fill in |
| `README.md` | Main deliverable | skeleton, fill in |

## First run on the GPU - ordered bring-up

Do not run everything at once. Each step isolates one failure mode.

```bash
# 1. Scene only. No ROS. Does it import, stand, and not explode?
/isaac-sim/python.sh scripts/build_scene.py --headless 1 --no-ros --frames 300

# 2. Do the drives bite?
/isaac-sim/python.sh scripts/build_scene.py --headless 1 --no-ros --spin-test --frames 600

# 3. Full ROS graph.
/isaac-sim/python.sh scripts/build_scene.py --headless 1

# 4. In the ros container, confirm topics:
scripts/verify_topics.sh

# 5. Teleoperate:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 6. Measure the scrub factor:
python3 scripts/measure_scrub.py
```

If a graph node fails to wire, `ros_graphs.py` automatically prints every
attribute on that node with its resolved type. Read that output rather than
guessing - it names the correct port in one run.
