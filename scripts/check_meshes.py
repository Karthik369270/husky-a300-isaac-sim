#!/usr/bin/env python3
"""
Verify the mesh pack against the URDF before importing anything.

Checks that every referenced mesh exists, and measures raw bounding boxes of
binary STLs to confirm which files are authored in millimetres. Catches a
missing or flattened meshes/ tree in one second instead of after a failed
import.

    python3 scripts/check_meshes.py [--urdf resources/husky_a300.urdf]
"""
import argparse
import os
import struct
import sys
import xml.etree.ElementTree as ET

p = argparse.ArgumentParser()
p.add_argument("--urdf", default="resources/husky_a300.urdf")
args = p.parse_args()

root_dir = os.path.dirname(os.path.abspath(args.urdf))
tree = ET.parse(args.urdf).getroot()

refs = {}
for link in tree.findall("link"):
    for tag in ("visual", "collision"):
        for el in link.findall(tag):
            mesh = el.find("geometry/mesh")
            if mesh is not None:
                refs.setdefault(mesh.get("filename"), mesh.get("scale"))


def stl_extents(path):
    """Return (dx, dy, dz) of a binary STL, or None if not binary."""
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        n = struct.unpack("<I", f.read(84)[80:84])[0]
        if size != 84 + n * 50:
            return None
        data = f.read(n * 50)
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for i in range(n):
        off = i * 50 + 12
        for v in range(3):
            xyz = struct.unpack("<3f", data[off + v * 12: off + v * 12 + 12])
            for k, c in enumerate(xyz):
                lo[k] = min(lo[k], c)
                hi[k] = max(hi[k], c)
    return tuple(hi[k] - lo[k] for k in range(3))


missing, scaled, unscaled_but_large = [], [], []
print(f"URDF references {len(refs)} unique meshes\n")
print(f"{'mesh':52s} {'scale':6s} {'max extent':>11s}")
print("-" * 74)

for rel, scale in sorted(refs.items()):
    full = os.path.join(root_dir, rel)
    if not os.path.exists(full):
        missing.append(rel)
        print(f"{rel:52s} {'yes' if scale else '-':6s} {'MISSING':>11s}")
        continue
    ext = stl_extents(full) if full.lower().endswith(".stl") else None
    mx = f"{max(ext):.3f}" if ext else "n/a"
    print(f"{rel:52s} {'yes' if scale else '-':6s} {mx:>11s}")
    if ext and max(ext) > 50.0:
        (scaled if scale else unscaled_but_large).append(rel)

print()
if missing:
    print(f"FAIL: {len(missing)} mesh(es) not found. Check the meshes/ tree is")
    print("      copied with its subdirectories intact, not flattened:")
    for m in missing:
        print(f"        {m}")
else:
    print("OK: every referenced mesh is present")

if scaled:
    print(f"\nOK: {len(scaled)} mesh(es) measure in the hundreds and carry a scale "
          f"attribute, as expected:")
    for m in scaled:
        print(f"      {m}")

if unscaled_but_large:
    print("\nWARNING: large extents but NO scale attribute - investigate:")
    for m in unscaled_but_large:
        print(f"      {m}")

sys.exit(1 if missing else 0)
