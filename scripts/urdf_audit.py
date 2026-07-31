#!/usr/bin/env python3
"""
Offline URDF audit: structure, mass budget, and millimetre-scaled meshes.
No simulator required.

    python3 scripts/urdf_audit.py [--urdf resources/husky_a300.urdf]
"""
import argparse
import os
import xml.etree.ElementTree as ET
from collections import defaultdict

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ap = argparse.ArgumentParser()
ap.add_argument("--urdf", default=os.path.join(REPO, "resources", "husky_a300.urdf"))
args = ap.parse_args()

if not os.path.exists(args.urdf):
    raise SystemExit(f"URDF not found: {args.urdf}")

root = ET.parse(args.urdf).getroot()
links = root.findall("link")
joints = root.findall("joint")

print(f"URDF: {args.urdf}\n")
print(f"robot name : {root.get('name')}")
print(f"links      : {len(links)}")
print(f"joints     : {len(joints)}")

by_type = defaultdict(int)
for j in joints:
    by_type[j.get("type")] += 1
print(f"joint types: {dict(by_type)}")

total = 0.0
massless = []
for l in links:
    inertial = l.find("inertial")
    if inertial is None:
        massless.append(l.get("name"))
        continue
    total += float(inertial.find("mass").get("value"))

print(f"\ntotal mass : {total:.3f} kg")
print(f"links with no <inertial> ({len(massless)}):")
for n in massless:
    print(f"    {n}")
print("\n  -> these are pure coordinate frames, including every sensor mount.")
print("     merge_fixed_joints MUST be False or they are deleted on import.")

print("\n--- meshes carrying a scale attribute ---")
found = 0
for l in links:
    for tag in ("visual", "collision"):
        for el in l.findall(tag):
            mesh = el.find("geometry/mesh")
            if mesh is not None and mesh.get("scale"):
                found += 1
                print(f"  {l.get('name'):38s} {tag:9s} "
                      f"scale={mesh.get('scale'):22s} {mesh.get('filename')}")
print(f"\n{found} scaled mesh entries - see docs/urdf_audit.md for measured extents")

print("\n--- driven joints ---")
for j in joints:
    if j.get("type") == "continuous":
        axis = j.find("axis")
        origin = j.find("origin")
        print(f"  {j.get('name'):26s} parent={j.find('parent').get('link'):24s} "
              f"axis={axis.get('xyz') if axis is not None else '-':10s} "
              f"origin={origin.get('xyz') if origin is not None else '-'}")
