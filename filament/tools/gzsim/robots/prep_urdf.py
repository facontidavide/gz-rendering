#!/usr/bin/env python3
"""prep_urdf.py IN.urdf OUT.urdf [--q j=val,...] [--stl] [--foot-z PREFIX] [--no-inertial]

--q  bake joint angles into the joint <origin> (so a static model at zero
     joint state shows that pose).
--no-inertial  replace every <inertial> by a dummy one (static model; sdformat rejects
     fer_link4 inertia and needs a mass on the root link).
--stl  drop <visual>, turn <collision> into <visual> (shows the STL meshes).
--pos LINK  print the link origin (root frame) at the baked pose.
--foot-z PREFIX  print -min z of link origins whose name contains PREFIX at the
     baked pose (model z offset that puts the feet on the ground).
"""
import sys, math, argparse
import xml.etree.ElementTree as ET
import numpy as np

def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])

def R2rpy(R):
    p = math.asin(-max(-1, min(1, R[2, 0])))
    if abs(R[2, 0]) > 0.99999:
        return 0.0, p, math.atan2(-R[0, 1], R[1, 1])
    return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])

def rod(axis, q):
    a = np.array(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q)*K + (1-math.cos(q))*K@K

ap = argparse.ArgumentParser()
ap.add_argument("src"); ap.add_argument("dst")
ap.add_argument("--q", default=""); ap.add_argument("--stl", action="store_true")
ap.add_argument("--foot-z", default=None)
ap.add_argument("--pos", default=None); ap.add_argument("--no-inertial", action="store_true")
a = ap.parse_args()
q = {k: float(v) for k, v in (kv.split("=") for kv in a.q.split(",") if kv)}

tree = ET.parse(a.src); root = tree.getroot()
T = {}  # link -> (R, t) in root frame
children = {}
for j in root.findall("joint"):
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    R = rpy2R(*rpy)
    name = j.get("name")
    if name in q:
        axis = [float(v) for v in j.find("axis").get("xyz").split()]
        R = R @ rod(axis, q[name])
        if o is None:
            o = ET.SubElement(j, "origin")
        o.set("rpy", " ".join(f"{v:.9g}" for v in R2rpy(R)))
        o.set("xyz", " ".join(f"{v:.9g}" for v in xyz))
    children.setdefault(j.find("parent").get("link"), []).append((j.find("child").get("link"), R, np.array(xyz)))
all_children = {c for v in children.values() for c, _, _ in v}
roots = [l.get("name") for l in root.findall("link") if l.get("name") not in all_children]
for r in roots:
    T[r] = (np.eye(3), np.zeros(3)); stack = [r]
    while stack:
        p = stack.pop(); Rp, tp = T[p]
        for c, R, t in children.get(p, []):
            T[c] = (Rp @ R, tp + Rp @ t); stack.append(c)

if a.no_inertial:
    for l in root.findall("link"):
        for i in l.findall("inertial"):
            l.remove(i)
        i = ET.SubElement(l, "inertial")
        ET.SubElement(i, "mass").set("value", "1")
        ET.SubElement(i, "inertia").attrib.update({k: v for k, v in zip(
            ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"], ["0.01", "0.01", "0.01", "0", "0", "0"])})
if a.stl:
    for l in root.findall("link"):
        for v in l.findall("visual"):
            l.remove(v)
        for c in l.findall("collision"):
            c.tag = "visual"
tree.write(a.dst, xml_declaration=True, encoding="utf-8")
if a.foot_z:
    print(-min(t[2] for n, (_, t) in T.items() if a.foot_z in n))
if a.pos:
    print(*np.round(T[a.pos][1], 3))
