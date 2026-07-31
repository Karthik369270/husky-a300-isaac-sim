#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Husky A300 -> Isaac Sim 5.1, placed in NVIDIA's warehouse environment.

Built entirely against the Isaac Sim Python API. Nothing here is done through
the GUI. Written against the v5.1.0 API:
  omni.kit.commands "URDFCreateImportConfig" / "URDFParseAndImportFile"

Usage:
    /isaac-sim/python.sh scripts/build_scene.py --headless 1
    /isaac-sim/python.sh scripts/build_scene.py --headless 0 --spin-test
"""
import argparse
import os
import sys

# ---------------------------------------------------------------- CLI --------
parser = argparse.ArgumentParser(description="Build the Husky A300 warehouse scene")
parser.add_argument("--headless", type=int, default=1, help="1 headless, 0 windowed")
parser.add_argument(
    "--urdf",
    default="/workspace/resources/husky_a300.urdf",
    help="Path to husky_a300.urdf",
)
parser.add_argument(
    "--env",
    default="/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    help="Warehouse USD, relative to the Isaac assets root",
)
parser.add_argument("--spawn", type=float, nargs=3, default=[0.0, 0.0, 0.30],
                    help="Robot spawn position in the warehouse (x y z)")
parser.add_argument("--wheel-damping", type=float, default=1.0e4,
                    help="Velocity-drive damping per wheel joint")
parser.add_argument("--wheel-max-force", type=float, default=1.0e5,
                    help="Max drive force per wheel joint")
parser.add_argument("--spin-test", action="store_true",
                    help="Spin the wheels open-loop to sanity check the drives")
parser.add_argument("--save-usd", default="",
                    help="Optional path to save the composed stage")
parser.add_argument("--frames", type=int, default=0,
                    help="Run N frames then exit (0 = run forever)")
parser.add_argument("--wheel-friction", type=float, default=0.0,
                    help="Static/dynamic friction on wheel colliders "
                         "(0 = leave PhysX defaults)")
parser.add_argument("--lidar", action="store_true",
                    help="Publish the Ouster OS1 point cloud (bonus)")
parser.add_argument("--camera", action="store_true",
                    help="Publish the ZED camera (bonus task)")
parser.add_argument("--no-ros", action="store_true",
                    help="Skip the ROS 2 graph (scene/articulation testing only)")
parser.add_argument("--domain-id", type=int, default=0, help="ROS_DOMAIN_ID")
parser.add_argument("--scrub-multiplier", type=float, default=1.75,
                    help="wheel_separation_multiplier for skid steer")
args, unknown = parser.parse_known_args()

# SimulationApp must be constructed before any other Isaac/omni import.
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {"renderer": "RaytracedLighting", "headless": bool(args.headless)}
)

import carb  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.stage import (  # noqa: E402
    is_stage_loading,
    create_new_stage,
    add_reference_to_stage,
)
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ros_graphs  # noqa: E402
import sensors  # noqa: E402

# The four driven joints, named exactly as in the URDF (see docs/topics.md).
WHEEL_JOINTS = [
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
]

PHYSICS_DT = 1.0 / 60.0
RENDER_DT = 1.0 / 60.0


def log(msg):
    print(f"[build_scene] {msg}", flush=True)


# ------------------------------------------------------- 1. warehouse --------
def load_warehouse():
    """
    Build a LOCAL stage and reference the warehouse into it.

    Opening the warehouse USD directly with open_stage() makes NVIDIA's
    read-only S3 layer the edit target. The URDF importer then tries to save
    the robot back into that layer, is refused, and the stage is invalidated.
    Referencing keeps the environment read-only while all authoring happens
    on a local anonymous root layer.
    """
    assets_root = get_assets_root_path()
    if assets_root is None:
        carb.log_error("Could not resolve the Isaac Sim assets root")
        simulation_app.close()
        sys.exit(1)

    env_usd = assets_root + args.env
    log("creating local stage")
    create_new_stage()
    simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    log("referencing environment: " + env_usd)
    add_reference_to_stage(usd_path=env_usd, prim_path="/World/Warehouse")

    simulation_app.update()
    simulation_app.update()
    while is_stage_loading():
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    log("environment referenced and loaded")
    return stage


# ------------------------------------------------------ 2. urdf import -------
def import_husky():
    urdf_path = os.path.abspath(args.urdf)
    if not os.path.exists(urdf_path):
        carb.log_error(f"URDF not found: {urdf_path}")
        simulation_app.close()
        sys.exit(1)

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")

    # Only the settings confirmed present in the 5.1 sample are assigned
    # unconditionally. Anything else is applied defensively so a missing
    # attribute in a point release cannot crash the build.
    import_config.merge_fixed_joints = False   # see docs/urdf_audit.md - MANDATORY
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True  # URDF ships real inertials
    import_config.fix_base = False              # floating base, it must drive
    import_config.distance_scale = 1.0          # URDF is already in metres

    optional = {
        "self_collision": False,
        "make_default_prim": False,   # warehouse is already the default prim
        "create_physics_scene": False,  # warehouse supplies one
        "density": 0.0,               # 0 = use the URDF's own masses
        "default_drive_strength": 0.0,
        "default_position_drive_damping": 0.0,
    }
    for key, value in optional.items():
        if hasattr(import_config, key):
            setattr(import_config, key, value)
            log(f"import_config.{key} = {value}")
        else:
            log(f"import_config has no '{key}' in this build, skipped")

    log(f"importing {urdf_path}")
    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        get_articulation_root=True,
    )
    if not status or not prim_path:
        carb.log_error("URDF import failed")
        simulation_app.close()
        sys.exit(1)

    log(f"imported, articulation root prim: {prim_path}")
    return prim_path


# ------------------------------------------------- 3. locate the joints ------
def find_wheel_joint_prims(stage):
    """
    Discover the four wheel joints by traversal rather than by hardcoded path.

    The URDF robot name is 'a300-00000'. USD prim names cannot contain a
    hyphen, so the importer sanitises it - the resulting prim path is not
    predictable from the URDF alone. Traversing avoids guessing.
    """
    wanted = set(WHEEL_JOINTS)
    found = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in wanted and prim.HasAPI(UsdPhysics.DriveAPI):
            found[name] = prim
        elif name in wanted and prim.IsA(UsdPhysics.RevoluteJoint):
            found[name] = prim

    missing = wanted - set(found)
    if missing:
        # Second pass, ignoring schema checks, in case the API is applied later.
        for prim in stage.Traverse():
            if prim.GetName() in missing:
                found[prim.GetName()] = prim
        missing = wanted - set(found)

    if missing:
        carb.log_error(f"wheel joints not found in stage: {sorted(missing)}")
        log("dumping candidate joint prims for debugging:")
        for prim in stage.Traverse():
            if "joint" in prim.GetName().lower():
                log(f"   {prim.GetPath()}  ({prim.GetTypeName()})")
        simulation_app.close()
        sys.exit(1)

    for n, p in found.items():
        log(f"joint {n:26s} -> {p.GetPath()}")
    return found


# --------------------------------------------------- 4. configure drives -----
def configure_velocity_drives(joint_prims):
    """
    Velocity control on all four wheels: stiffness 0, damping high.

    Note the Isaac Sim sample sets TargetVelocity in DEGREES per second, not
    radians. The differential controller must convert accordingly.
    """
    drives = {}
    for name, prim in joint_prims.items():
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            log(f"applied a new angular DriveAPI to {name}")

        drive.CreateStiffnessAttr().Set(0.0)
        drive.CreateDampingAttr().Set(args.wheel_damping)
        drive.CreateMaxForceAttr().Set(args.wheel_max_force)
        drive.CreateTargetVelocityAttr().Set(0.0)
        drives[name] = drive
        log(f"drive {name:26s} stiffness=0 damping={args.wheel_damping:g} "
            f"maxForce={args.wheel_max_force:g}")
    return drives


def apply_wheel_friction(stage, robot_prim_path):
    """
    Give the wheel colliders a rubber-like friction material.

    PhysX defaults are around 0.5, which is closer to plastic than to outdoor
    tyres on concrete. Measured scrub with the defaults required a
    wheel_separation_multiplier near 7, against Clearpath's hardware value of
    1.75 - a sign the friction model, not the kinematics, was the problem.
    """
    from pxr import UsdShade, PhysxSchema as _Px
    mu = args.wheel_friction
    mat_path = Sdf.Path("/World/PhysicsMaterials/WheelRubber")
    UsdShade.Material.Define(stage, mat_path)
    mat_prim = stage.GetPrimAtPath(mat_path)
    phys_mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
    phys_mat.CreateStaticFrictionAttr().Set(mu)
    phys_mat.CreateDynamicFrictionAttr().Set(mu)
    phys_mat.CreateRestitutionAttr().Set(0.0)
    log(f"wheel friction material mu={mu}")

    n = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "wheel_link" in path and "collisions" in path:
            binding = UsdShade.MaterialBindingAPI.Apply(prim)
            binding.Bind(UsdShade.Material(stage.GetPrimAtPath(mat_path)),
                         bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                         materialPurpose="physics")
            n += 1
    log(f"friction material bound to {n} wheel collider prims")


# ------------------------------------------------------- 5. placement --------
def place_robot(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        carb.log_error(f"cannot place robot, invalid prim: {prim_path}")
        return
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    x, y, z = args.spawn
    xform.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))
    log(f"robot placed at ({x}, {y}, {z})")


# ------------------------------------------------------ 6. physics tune ------
def tune_physics(stage):
    scene_path = None
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            scene_path = prim.GetPath()
            break
    if scene_path is None:
        log("no physics scene in the environment, creating one")
        UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        scene_path = Sdf.Path("/physicsScene")

    scene = UsdPhysics.Scene.Get(stage, scene_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(scene_path))
    physx = PhysxSchema.PhysxSceneAPI.Get(stage, scene_path)
    physx.CreateEnableCCDAttr(True)
    physx.CreateEnableStabilizationAttr(True)
    physx.CreateEnableGPUDynamicsAttr(False)
    physx.CreateBroadphaseTypeAttr("MBP")
    physx.CreateSolverTypeAttr("TGS")
    log(f"physics scene configured at {scene_path}")


# --------------------------------------------------- find base_link ---------
def find_link_prim(stage, link_name):
    """Locate a link prim by name. The robot prim path is not predictable
    because the URDF robot name 'a300-00000' contains a hyphen."""
    for prim in stage.Traverse():
        if prim.GetName() == link_name:
            return str(prim.GetPath())
    return None


# ------------------------------------------------------------- main ----------
def main():
    if not args.no_ros:
        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        log("isaacsim.ros2.bridge enabled")

    stage = load_warehouse()
    prim_path = import_husky()

    # Re-fetch: the import mutates the stage.
    stage = omni.usd.get_context().get_stage()

    place_robot(stage, prim_path)
    tune_physics(stage)
    if args.wheel_friction > 0.0:
        apply_wheel_friction(stage, prim_path)
    joint_prims = find_wheel_joint_prims(stage)
    drives = configure_velocity_drives(joint_prims)

    controller = None
    if not args.no_ros:
        base_link = find_link_prim(stage, "base_link")
        if base_link is None:
            carb.log_error("base_link prim not found, cannot build ROS graph")
        else:
            log(f"base_link prim: {base_link}")
            ros_graphs.build_ros_graph(
                stage, prim_path, base_link, domain_id=args.domain_id
            )
            controller = ros_graphs.SkidSteerController(
                drives, multiplier=args.scrub_multiplier
            )
            if args.camera:
                sensors.setup_camera(stage, prim_path)
            if args.lidar:
                sensors.setup_lidar(stage, prim_path)

    if args.save_usd:
        stage.Export(args.save_usd)
        log(f"stage exported to {args.save_usd}")

    sim = SimulationContext(
        physics_dt=PHYSICS_DT, rendering_dt=RENDER_DT, stage_units_in_meters=1.0
    )
    sim.initialize_physics()
    sim.play()
    sim.step()

    art = Articulation(prim_path)
    art.initialize()
    if not art.is_physics_handle_valid():
        carb.log_error(f"{prim_path} did not resolve to a valid articulation")
    else:
        log(f"articulation OK: {prim_path}")
        try:
            log(f"dof names: {art.dof_names}")
            log(f"num dof:   {art.num_dof}")
        except Exception as exc:  # attribute names vary slightly across builds
            log(f"(could not read dof metadata: {exc!r})")

    # Let it settle so we can see whether it rests or explodes.
    log("settling for 120 frames ...")
    for _ in range(120):
        sim.step(render=True)

    if args.spin_test:
        log("spin test: 90 deg/s on the left pair, -90 on the right")
        for name, drive in drives.items():
            sign = 1.0 if "left" in name else -1.0
            drive.GetTargetVelocityAttr().Set(90.0 * sign)  # degrees/second

    log("scene ready - publishing. Drive it with:")
    log("  ros2 run teleop_twist_keyboard teleop_twist_keyboard")
    frame = 0
    while simulation_app.is_running():
        sim.step(render=True)

        # Pull the latest /cmd_vel and convert to four wheel velocities.
        if controller is not None and not args.spin_test:
            v, omega = ros_graphs.read_cmd_vel()
            controller.apply(v, omega)

        frame += 1
        if args.frames and frame >= args.frames:
            break

    if controller is not None:
        controller.stop()
    sim.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
