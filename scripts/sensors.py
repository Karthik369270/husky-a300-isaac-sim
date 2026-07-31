#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Sensor graphs: ZED 2 stereo camera (RGB + depth + camera_info) and IMU.

The URDF supplies only empty coordinate frames at the sensor mount points -
no sensors are created by the import. They are constructed here at those
frames, in code.

Two things the frames dictate (see docs/urdf_audit.md):

  * The camera faces the REAR of the robot, over the cargo bed. This is how
    the robot is specified and is deliberately not corrected.
  * A USD camera prim looks down its own -Z with +Y up. A ROS optical frame is
    +Z forward, +X right, +Y down. The camera prim therefore needs a fixed
    rotation relative to camera_0_left_camera_frame_optical.
"""
import omni.graph.core as og
from pxr import Gf, Sdf, UsdGeom

from ros_graphs import GRAPH_PATH, describe_node, log, set_targets

CAMERA_PRIM = "/World/husky_zed_left"
CAMERA_FRAME = "camera_0_left_camera_frame_optical"
IMU_FRAME = "imu_0_link"

RESOLUTION = (1280, 720)


def create_camera_prim(stage, optical_frame_path):
    """
    Create a USD camera under the optical frame.

    USD camera: -Z forward, +Y up.
    ROS optical: +Z forward, +X right, +Y down.

    Rotating 180 degrees about X maps one to the other: it flips Z (so the
    prim's -Z aligns with the frame's +Z) and flips Y (so +Y up becomes
    +Y down). X is unchanged, keeping "right" consistent.
    """
    path = Sdf.Path(CAMERA_PRIM)
    cam = UsdGeom.Camera.Define(stage, path)
    prim = cam.GetPrim()

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddRotateXOp().Set(180.0)

    # ZED 2 left lens, approximate
    cam.CreateFocalLengthAttr(2.12)
    cam.CreateHorizontalApertureAttr(5.76)
    cam.CreateVerticalApertureAttr(3.24)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))

    log(f"camera prim created at {CAMERA_PRIM} (180 deg about X for ROS optical)")
    return str(path)


def reparent_camera(stage, optical_frame_path):
    """Place the camera prim so it inherits the optical frame's pose."""
    from pxr import Sdf as _Sdf
    import omni.kit.commands
    target = f"{optical_frame_path}/husky_zed_left"
    try:
        omni.kit.commands.execute(
            "MovePrim", path_from=CAMERA_PRIM, path_to=target
        )
        log(f"camera reparented to {target}")
        return target
    except Exception as exc:
        log(f"could not reparent camera ({exc!r}); leaving at {CAMERA_PRIM}")
        return CAMERA_PRIM


def build_camera_graph(stage, camera_prim_path):
    """RGB, depth and camera_info publishers driven off one render product."""
    keys = og.Controller.Keys
    log("building camera graph")

    try:
        og.Controller.edit(
            GRAPH_PATH,
            {
                keys.CREATE_NODES: [
                    ("CreateRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("CamRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CamDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CamInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "CreateRP.inputs:execIn"),
                    ("CreateRP.outputs:execOut", "CamRgb.inputs:execIn"),
                    ("CreateRP.outputs:execOut", "CamDepth.inputs:execIn"),
                    ("CreateRP.outputs:execOut", "CamInfo.inputs:execIn"),
                    ("CreateRP.outputs:renderProductPath",
                     "CamRgb.inputs:renderProductPath"),
                    ("CreateRP.outputs:renderProductPath",
                     "CamDepth.inputs:renderProductPath"),
                    ("CreateRP.outputs:renderProductPath",
                     "CamInfo.inputs:renderProductPath"),
                    ("Context.outputs:context", "CamRgb.inputs:context"),
                    ("Context.outputs:context", "CamDepth.inputs:context"),
                    ("Context.outputs:context", "CamInfo.inputs:context"),
                ],
                keys.SET_VALUES: [
                    ("CreateRP.inputs:width", RESOLUTION[0]),
                    ("CreateRP.inputs:height", RESOLUTION[1]),
                    ("CamRgb.inputs:topicName", "/camera/image_raw"),
                    ("CamRgb.inputs:type", "rgb"),
                    ("CamRgb.inputs:frameId", CAMERA_FRAME),
                    ("CamDepth.inputs:topicName", "/camera/depth"),
                    ("CamDepth.inputs:type", "depth"),
                    ("CamDepth.inputs:frameId", CAMERA_FRAME),
                    ("CamInfo.inputs:topicName", "/camera/camera_info"),
                    ("CamInfo.inputs:frameId", CAMERA_FRAME),
                ],
            },
        )
    except Exception as exc:
        log(f"camera graph FAILED: {exc!r}")
        for n in ("CreateRP", "CamRgb", "CamInfo"):
            describe_node(f"{GRAPH_PATH}/{n}")
        return False

    set_targets(stage, f"{GRAPH_PATH}/CreateRP", "inputs:cameraPrim",
                [camera_prim_path])
    log("camera graph built")
    return True


def setup_camera(stage, robot_prim_path):
    """Locate the optical frame, build the camera there, wire the publishers."""
    optical = None
    for prim in stage.Traverse():
        if prim.GetName() == CAMERA_FRAME:
            optical = str(prim.GetPath())
            break
    if optical is None:
        log(f"optical frame {CAMERA_FRAME} not found - was merge_fixed_joints True?")
        return False

    log(f"optical frame: {optical}")
    log("note: this camera faces the REAR of the robot, as specified in topics.md")
    create_camera_prim(stage, optical)
    cam_path = reparent_camera(stage, optical)
    return build_camera_graph(stage, cam_path)


# ============================================================== LIDAR ========
LIDAR_PRIM = "/World/husky_ouster_os1"
LIDAR_FRAME = "lidar3d_0_sensor_link"


def create_lidar_prim(stage, sensor_frame_path):
    """
    Create an RTX LiDAR at the Ouster OS1 mount frame.

    The URDF gives only an empty frame here - the sensor itself has to be
    created in the simulator and wired up. Isaac Sim ships an OS1 config
    which is used if available, otherwise a generic rotary profile.
    """
    import omni.kit.commands

    target = f"{sensor_frame_path}/husky_ouster_os1"

    # This build ships no Ouster OS1 profile (checked: no OS1/Ouster json in
    # omni.sensors.nv.common/data/lidar). Example_Rotary is a generic spinning
    # 3D profile and is used instead; the mount pose and frame_id remain those
    # of the OS1 as specified. Noted as a limitation in the README.
    for cfg in ("Example_Rotary", "Example_Rotary_BEAMS", "OS1_REV6_128ch10hz1024res"):
        try:
            result, prim = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path="husky_ouster_os1",
                parent=sensor_frame_path,
                config=cfg,
                translation=Gf.Vec3d(0.0, 0.0, 0.0),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            )
            if result and prim is not None:
                # Use the prim the command actually created. The path is not
                # reliably predictable from the arguments, and pointing the
                # render product at a non-existent path yields
                # "Render product not attached to RTX Lidar".
                actual = str(prim.GetPath()) if hasattr(prim, "GetPath") else str(prim)
                log(f"RTX LiDAR created at {actual} with config '{cfg}'")
                return actual
        except Exception as exc:
            log(f"lidar config '{cfg}' failed: {exc!r}")

    log("could not create RTX LiDAR with any known config")
    return None


def build_lidar_graph(stage, lidar_prim_path):
    """Publish the LiDAR return as sensor_msgs/PointCloud2 on /points."""
    keys = og.Controller.Keys
    log("building lidar graph")

    try:
        og.Controller.edit(
            GRAPH_PATH,
            {
                keys.CREATE_NODES: [
                    ("CreateRPLidar", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("LidarPub", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ],
                keys.CONNECT: [
                    (f"{GRAPH_PATH}/OnTick.outputs:tick",
                     f"{GRAPH_PATH}/CreateRPLidar.inputs:execIn"),
                    (f"{GRAPH_PATH}/CreateRPLidar.outputs:execOut",
                     f"{GRAPH_PATH}/LidarPub.inputs:execIn"),
                    (f"{GRAPH_PATH}/CreateRPLidar.outputs:renderProductPath",
                     f"{GRAPH_PATH}/LidarPub.inputs:renderProductPath"),
                    (f"{GRAPH_PATH}/Context.outputs:context",
                     f"{GRAPH_PATH}/LidarPub.inputs:context"),
                ],
                keys.SET_VALUES: [
                    (f"{GRAPH_PATH}/LidarPub.inputs:topicName", "/points"),
                    (f"{GRAPH_PATH}/LidarPub.inputs:frameId", LIDAR_FRAME),
                    (f"{GRAPH_PATH}/LidarPub.inputs:type", "point_cloud"),
                ],
            },
        )
    except Exception as exc:
        log(f"lidar graph FAILED: {exc!r}")
        for n in ("CreateRPLidar", "LidarPub"):
            describe_node(f"{GRAPH_PATH}/{n}")
        return False

    set_targets(stage, f"{GRAPH_PATH}/CreateRPLidar", "inputs:cameraPrim",
                [lidar_prim_path])
    log("lidar graph built")
    return True


def setup_lidar(stage, robot_prim_path):
    """Locate the LiDAR mount frame, create the sensor, wire the publisher."""
    frame = None
    for prim in stage.Traverse():
        if prim.GetName() == LIDAR_FRAME:
            frame = str(prim.GetPath())
            break
    if frame is None:
        log(f"lidar frame {LIDAR_FRAME} not found")
        return False

    log(f"lidar mount frame: {frame}")
    lidar_path = create_lidar_prim(stage, frame)
    if lidar_path is None:
        return False
    return build_lidar_graph(stage, lidar_path)
