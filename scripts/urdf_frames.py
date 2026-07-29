#!/usr/bin/env python3
"""
Resolve wheel and sensor frames through the full kinematic chain to base_link.
Requires numpy.

    python3 scripts/urdf_frames.py [--urdf resources/husky_a300.urdf]
"""
import argparse
import os
import xml.etree.ElementTree as ET

try:
    import numpy as np
except ImportError:
    raise SystemExit("numpy required:  pip install numpy  (or apt install python3-numpy)")

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ap = argparse.ArgumentParser()
ap.add_argument("--urdf", default=os.path.join(REPO, "resources", "husky_a300.urdf"))
args = ap.parse_args()

if not os.path.exists(args.urdf):
    raise SystemExit(f"URDF not found: {args.urdf}")

root = ET.parse(args.urdf).getroot()


def rpy_to_R(rpy):
    x, y, z = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
    Ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    Rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


parent, local = {}, {}
for j in root.findall("joint"):
    child = j.find("child").get("link")
    parent[child] = j.find("parent").get("link")
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0")).split()] if o is not None else [0, 0, 0]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0")).split()] if o is not None else [0, 0, 0]
    local[child] = (np.array(xyz), rpy_to_R(rpy))


def to_base(link):
    T = np.eye(4)
    while link in parent:
        xyz, R = local[link]
        L = np.eye(4)
        L[:3, :3] = R
        L[:3, 3] = xyz
        T = L @ T
        link = parent[link]
    return T


WHEELS = ["front_left_wheel_link", "front_right_wheel_link",
          "rear_left_wheel_link", "rear_right_wheel_link"]

print("=== WHEEL POSITIONS IN base_link (m) ===")
pos = {}
for w in WHEELS:
    T = to_base(w)
    pos[w] = T[:3, 3]
    print(f"  {w:26s} x={T[0,3]:+.4f}  y={T[1,3]:+.4f}  z={T[2,3]:+.4f}")

track = abs(pos[WHEELS[0]][1] - pos[WHEELS[1]][1])
wheelbase = abs(pos[WHEELS[0]][0] - pos[WHEELS[2]][0])
print(f"\n  measured track     = {track:.4f} m")
print(f"  measured wheelbase = {wheelbase:.4f} m")
print(f"  effective track at 1.75x = {track * 1.75:.4f} m")

print("\n=== SENSOR FRAMES IN base_link ===")
for s in ["lidar3d_0_sensor_link", "imu_0_link",
          "camera_0_left_camera_frame", "camera_0_left_camera_frame_optical"]:
    T = to_base(s)
    R = T[:3, :3]
    print(f"\n  {s}")
    print(f"    xyz = [{T[0,3]:+.5f}, {T[1,3]:+.5f}, {T[2,3]:+.5f}]")
    print(f"    +X -> {np.round(R @ np.array([1, 0, 0]), 4)}")
    print(f"    +Y -> {np.round(R @ np.array([0, 1, 0]), 4)}")
    print(f"    +Z -> {np.round(R @ np.array([0, 0, 1]), 4)}")
