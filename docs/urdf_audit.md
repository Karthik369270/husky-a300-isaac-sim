# URDF audit

Run: `python3 scripts/urdf_audit.py` and `python3 scripts/urdf_frames.py`

Performed before any simulator work, to know what to expect on import.

## Structure

| Property | Value |
|---|---|
| robot name | `a300-00000` |
| links | 35 |
| joints | 34 (30 fixed, 4 continuous) |
| total mass | **108.268 kg** (matches the ~108 kg stated in RESOURCES.md) |
| links without an `<inertial>` block | 12 |

The 12 massless links are pure coordinate frames:
`base_footprint`, `base_link`, `lidar3d_0_sensor_link`, `camera_0_baro_link`,
`camera_0_camera_link`, `camera_0_left_camera_frame`,
`camera_0_left_camera_frame_optical`, `camera_0_mag_link`,
`camera_0_right_camera_frame`, `camera_0_right_camera_frame_optical`,
`camera_0_temp_left_link`, `camera_0_temp_right_link`.

**Consequence for the import:** `merge_fixed_joints` must be **False**. The default
merges fixed-joint children into their parent rigid body, which would remove
`lidar3d_0_sensor_link`, `camera_0_left_camera_frame_optical` and `imu_0_link`
from the stage - leaving the sensors nowhere to mount and producing a `/tf` tree
that does not match the frame tree specified in `topics.md`.

## The five millimetre-scaled meshes

RESOURCES.md warns that five meshes carry `scale="0.001 0.001 0.001"`. Located:

| Link | Element | Mesh |
|---|---|---|
| `enclosure_access_panels_link` | visual | `observer_access_panels.stl` |
| `enclosure_enclosure_link` | visual | `observer_enclosure.stl` |
| `enclosure_enclosure_link` | collision | `observer_enclosure.stl` |
| `sensor_arch_link` | visual | `observer_arch.stl` |
| `sensor_arch_link` | collision | `observer_arch.stl` |

All belong to the enclosure/arch assembly. Note that two of the five are
*collision* geometry - a visual-only check would miss them.

## Wheel geometry (computed through the full kinematic chain)

`base_link` to a wheel is four transforms, not one - the wheels hang off the
suspension beam and motor links.

| Wheel | x | y | z |
|---|---|---|---|
| `front_left_wheel_link` | +0.2560 | +0.2829 | +0.0291 |
| `front_right_wheel_link` | +0.2560 | -0.2639 | +0.0291 |
| `rear_left_wheel_link` | -0.2560 | +0.2829 | +0.0291 |
| `rear_right_wheel_link` | -0.2560 | -0.2639 | +0.0291 |

All four driven joints rotate about `axis = 0 1 0`.

- measured track: **0.5468 m**
- measured wheelbase: **0.5120 m**

Two observations. The track is **not symmetric about the centreline**
(+0.2829 left, -0.2639 right, a 19 mm offset), and 0.5468 m does not match the
0.562 m quoted in RESOURCES.md. The URDF geometry is taken as authoritative
here. With the 1.75 multiplier this gives an effective track of 0.9569 m rather
than the 0.984 m quoted - see `docs/scrub_measurement.md` for the measured value.

## Sensor frames, expressed in `base_link`

| Frame | xyz (m) | orientation |
|---|---|---|
| `lidar3d_0_sensor_link` | `[-0.40710, 0.00000, 0.91495]` | identity |
| `imu_0_link` | `[ 0.05900, 0.00000, 0.16128]` | identity |
| `camera_0_left_camera_frame` | `[-0.40489, -0.06000, 0.76447]` | ~180 deg about Z |

### The camera faces the rear - confirmed, and left as specified

`topics.md` states the ZED is mounted facing backwards over the cargo bed, and
asks that this be noted rather than silently corrected. Confirmed numerically:
the local +X axis of `camera_0_left_camera_frame` points along
`[-0.9997, -0.0102, -0.0230]` in `base_link` - directly rearward, with a small
downward tilt. **No correction has been applied.**

### Optical frame convention

`camera_0_left_camera_frame_optical` follows the ROS convention exactly:

| Optical axis | Direction in `base_link` | Meaning |
|---|---|---|
| +Z | `[-0.9997, -0.0102, -0.0230]` | forward, i.e. the view direction (rearward on the robot) |
| +X | `[-0.0102, +0.9999, +0.0003]` | camera-right (robot-left, since the camera faces rear) |
| +Y | `[+0.0230, +0.0005, -0.9997]` | camera-down |

Z forward, X right, Y down. A USD camera prim uses -Z forward and +Y up, so the
camera prim needs a fixed rotation relative to this frame; image topics are
published in the optical frame, not the camera body frame.

## Mesh pack verification

Run: `python3 scripts/check_meshes.py`

The URDF references **18 unique meshes across three directories**. The tree
structure matters - paths are relative and resolved literally by the importer:

| Directory | Files referenced |
|---|---|
| `meshes/clearpath_platform/` | 14 |
| `meshes/clearpath_sensors/` | 3 (`os1_base.dae`, `os1_halo.dae`, `os1_lidar.dae`) |
| `meshes/zed/` | 1 (`zed2.stl`) |

RESOURCES.md counts 21 files in the pack against these 18 references; the
remainder are texture images referenced from inside the COLLADA files.

### Millimetre scaling confirmed by measurement

Rather than trusting the `scale` attribute, raw bounding boxes were measured
directly from the binary STL data:

| Mesh | Triangles | Raw extents |
|---|---|---|
| `chassis_collision.stl` | 196 | 0.899 x 0.461 x 0.230 |
| `motor.stl` | 46604 | 0.126 x 0.126 x 0.125 |
| `outdoor_left.stl` | 95626 | 0.336 x 0.120 x 0.336 |
| `outdoor_right.stl` | 95626 | 0.336 x 0.120 x 0.336 |
| `suspension_beam.stl` | 3004 | 0.586 x 0.010 x 0.104 |
| `suspension_spacer.stl` | 748 | 0.076 x 0.016 x 0.064 |
| `bumper_collision.stl` | 106 | 0.128 x 0.378 x 0.033 |
| **`observer_access_panels.stl`** | 56 | **858.9 x 459.9 x 179.5** |
| **`observer_arch.stl`** | 116 | **175.7 x 650.0 x 445.4** |
| **`observer_enclosure.stl`** | 140 | **971.4 x 452.9 x 186.7** |

Three orders of magnitude apart, and exactly the three meshes carrying
`scale="0.001 0.001 0.001"`. Sanity check on the metre-scale files:
`outdoor_left.stl` measures 0.336 m across against a stated wheel radius of
0.1651 m (0.330 m diameter plus tread) - correct as authored.

If the scale attribute were dropped on import, `observer_arch.stl` alone would
render 650 m tall.

Incidental note: the three millimetre meshes are very low polygon count
(56-140 triangles, essentially boxes) compared with the 95k-triangle wheels, so
using them directly as convex collision geometry is inexpensive.
