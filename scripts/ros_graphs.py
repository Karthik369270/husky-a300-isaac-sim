#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
ROS 2 bridge OmniGraph construction for the Husky A300, plus the skid-steer
controller that turns /cmd_vel into four wheel velocities.

Built entirely from the Isaac Sim Python API using og.Controller.edit - no part
of this graph is authored in the GUI.

Publishes:                                  Subscribes:
    /clock          rosgraph_msgs/Clock         /cmd_vel   geometry_msgs/Twist
    /tf             tf2_msgs/TFMessage
    /tf_static      tf2_msgs/TFMessage
    /joint_states   sensor_msgs/JointState
    /odom           nav_msgs/Odometry   (frame_id odom, child_frame_id base_link)

Node type strings below were taken from the registered node list of the
v5.1.0 source tree, not from documentation.
"""
import math

import omni.graph.core as og
from pxr import UsdPhysics

GRAPH_PATH = "/ROS2Graph"

# Driven joints, named exactly as in the URDF.
WHEEL_JOINTS = [
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
]
LEFT_JOINTS = ["front_left_wheel_joint", "rear_left_wheel_joint"]
RIGHT_JOINTS = ["front_right_wheel_joint", "rear_right_wheel_joint"]

# Geometry, measured from the URDF chain (see docs/urdf_audit.md).
WHEEL_RADIUS = 0.1651     # m, from RESOURCES.md
PHYSICAL_TRACK = 0.5468   # m, measured - note this differs from the 0.562 quoted


def log(msg):
    print(f"[ros_graphs] {msg}", flush=True)


# --------------------------------------------------------------- helpers ----
def describe_node(node_path):
    """
    Print every attribute on a node. Called automatically when wiring fails so
    a mismatched port name is diagnosed in one run instead of several.
    """
    try:
        node = og.Controller.node(node_path)
        log(f"attributes on {node_path}:")
        for attr in node.get_attributes():
            log(f"    {attr.get_name()}  ({attr.get_resolved_type().get_type_name()})")
    except Exception as exc:
        log(f"could not introspect {node_path}: {exc!r}")


def set_targets(stage, node_path, attribute, target_paths):
    """
    Assign a USD relationship on an OmniGraph node.

    Relationship attributes cannot be set through SET_VALUES the way plain
    values can, so this is done directly on the USD prim. Tries the Isaac
    helper first and falls back to raw USD.
    """
    try:
        from isaacsim.core.utils.prims import set_targets as _isaac_set_targets
        _isaac_set_targets(
            prim=stage.GetPrimAtPath(node_path),
            attribute=attribute,
            target_prim_paths=list(target_paths),
        )
        log(f"set {node_path}.{attribute} -> {list(target_paths)}")
        return True
    except Exception as exc:
        log(f"isaac set_targets failed ({exc!r}), falling back to raw USD")

    try:
        prim = stage.GetPrimAtPath(node_path)
        rel = prim.GetRelationship(attribute)
        if not rel:
            rel = prim.CreateRelationship(attribute)
        rel.SetTargets(list(target_paths))
        log(f"set {node_path}.{attribute} -> {list(target_paths)} (raw USD)")
        return True
    except Exception as exc:
        log(f"FAILED to set {node_path}.{attribute}: {exc!r}")
        describe_node(node_path)
        return False


# ------------------------------------------------------------ graph build ---
def build_ros_graph(stage, robot_prim_path, base_link_path, domain_id=0):
    """
    Create the ROS 2 publisher/subscriber action graph.

    robot_prim_path : articulation root, as returned by URDFParseAndImportFile
    base_link_path  : prim path of base_link, the odometry chassis frame
    """
    keys = og.Controller.Keys

    log(f"building graph at {GRAPH_PATH}")
    log(f"  articulation : {robot_prim_path}")
    log(f"  chassis      : {base_link_path}")

    (graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),

                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("PublishTFStatic", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),

                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),

                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ],
            keys.CONNECT: [
                # Everything ticks off playback so it publishes every frame.
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishTFStatic.inputs:execIn"),
                ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "SubscribeTwist.inputs:execIn"),

                # Simulation time drives every stamp. This is what makes
                # use_sim_time meaningful downstream.
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTFStatic.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),

                # Shared ROS 2 context so every node uses one domain.
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "PublishTF.inputs:context"),
                ("Context.outputs:context", "PublishTFStatic.inputs:context"),
                ("Context.outputs:context", "PublishOdom.inputs:context"),
                ("Context.outputs:context", "PublishOdomTF.inputs:context"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),

                # Odometry: compute once, publish as both /odom and odom->base_link TF.
                ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PublishOdomTF.inputs:execIn"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("ComputeOdom.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdom.outputs:orientation", "PublishOdomTF.inputs:rotation"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", domain_id),

                ("PublishClock.inputs:topicName", "/clock"),
                ("PublishJointState.inputs:topicName", "/joint_states"),
                ("PublishTF.inputs:topicName", "/tf"),
                ("PublishTFStatic.inputs:topicName", "/tf_static"),
                ("PublishTFStatic.inputs:staticPublisher", True),
                ("PublishOdom.inputs:topicName", "/odom"),

                # topics.md requires exactly these frame ids.
                ("PublishOdom.inputs:odomFrameId", "odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                ("PublishOdomTF.inputs:parentFrameId", "odom"),
                ("PublishOdomTF.inputs:childFrameId", "base_link"),
                ("PublishOdomTF.inputs:topicName", "/tf"),

                ("SubscribeTwist.inputs:topicName", "/cmd_vel"),
            ],
        },
    )

    # Relationships must be assigned separately.
    set_targets(stage, f"{GRAPH_PATH}/PublishJointState", "inputs:targetPrim",
                [robot_prim_path])
    set_targets(stage, f"{GRAPH_PATH}/ComputeOdom", "inputs:chassisPrim",
                [base_link_path])
    set_targets(stage, f"{GRAPH_PATH}/PublishTF", "inputs:targetPrims",
                [robot_prim_path])
    set_targets(stage, f"{GRAPH_PATH}/PublishTFStatic", "inputs:targetPrims",
                [robot_prim_path])

    log("graph built")
    return graph


def read_cmd_vel():
    """Read the latest Twist off the subscriber node. Returns (v, omega)."""
    try:
        lin = og.Controller.attribute(
            f"{GRAPH_PATH}/SubscribeTwist.outputs:linearVelocity").get()
        ang = og.Controller.attribute(
            f"{GRAPH_PATH}/SubscribeTwist.outputs:angularVelocity").get()
        return float(lin[0]), float(ang[2])
    except Exception as exc:
        log(f"could not read /cmd_vel: {exc!r}")
        describe_node(f"{GRAPH_PATH}/SubscribeTwist")
        return 0.0, 0.0


# ------------------------------------------------------ skid steer control --
class SkidSteerController:
    """
    Four-wheel skid-steer kinematics.

    Two fixed axles cannot rotate without the tyres scrubbing sideways, so
    textbook differential kinematics under-rotates. Clearpath compensate on
    hardware with wheel_separation_multiplier: 1.75. RESOURCES.md notes that
    whether the same figure holds in simulation is worth measuring - see
    scripts/measure_scrub.py and docs/scrub_measurement.md.

    Drive targets are written in DEGREES per second: that is the unit
    UsdPhysics.DriveAPI expects for an angular drive.
    """

    def __init__(self, drives, wheel_radius=WHEEL_RADIUS,
                 track=PHYSICAL_TRACK, multiplier=1.75, max_wheel_rad_s=12.0):
        self.drives = drives
        self.r = wheel_radius
        self.track = track
        self.multiplier = multiplier
        self.max_wheel_rad_s = max_wheel_rad_s
        log(f"skid steer: r={self.r} track={self.track} "
            f"multiplier={self.multiplier} effective_track={self.effective_track:.4f}")

    @property
    def effective_track(self):
        return self.track * self.multiplier

    def wheel_speeds(self, v, omega):
        """Return (left_rad_s, right_rad_s) for a body twist."""
        half = self.effective_track / 2.0
        v_left = v - omega * half
        v_right = v + omega * half
        return v_left / self.r, v_right / self.r

    def apply(self, v, omega):
        left, right = self.wheel_speeds(v, omega)
        left = max(-self.max_wheel_rad_s, min(self.max_wheel_rad_s, left))
        right = max(-self.max_wheel_rad_s, min(self.max_wheel_rad_s, right))

        left_deg = math.degrees(left)
        right_deg = math.degrees(right)

        for name in LEFT_JOINTS:
            if name in self.drives:
                self.drives[name].GetTargetVelocityAttr().Set(left_deg)
        for name in RIGHT_JOINTS:
            if name in self.drives:
                self.drives[name].GetTargetVelocityAttr().Set(right_deg)
        return left, right

    def stop(self):
        for drive in self.drives.values():
            drive.GetTargetVelocityAttr().Set(0.0)
