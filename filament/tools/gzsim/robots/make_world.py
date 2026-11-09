#!/usr/bin/env python3
"""make_world.py ROBOT ENGINE OUT.sdf [MODEL_Z]  -> world from world_template.sdf"""
import sys, math, os

# model name -> (sdf file in /out, "x y z" at ground, z set from argv for anymal)
# camera: (name, eye, target, hfov)
ROBOTS = {
    "anymal": dict(
        models=[("anymal", (0, 0, None), "file:///out/anymal.sdf")],
        cams=[("full", (2.0, -1.7, 1.0), (0, 0, 0.35), 0.85),
              ("face", (1.3, -0.7, 0.85), (0.42, 0, 0.62), 0.9),
              ("lidar", (-1.2, 0.9, 1.15), (-0.3, 0, 0.7), 0.9)]),
    "franka": dict(
        models=[("franka", (0, 0.55, 0), "file:///out/franka.sdf"),
                ("franka_stl", (0, -0.55, 0), "file:///out/franka_stl.sdf")],
        cams=[("full", (2.8, 0, 1.3), (0.2, 0, 0.45), 0.9),
              ("wrist_dae", (0.79, 0.14, 0.85), (0.29, 0.64, 0.6), 0.8),
              ("wrist_stl", (0.79, -0.96, 0.85), (0.29, -0.46, 0.6), 0.8)]),
}

def lookat(eye, tgt):
    d = [t - e for e, t in zip(eye, tgt)]
    return math.asin(-d[2] / math.sqrt(sum(v * v for v in d))), math.atan2(d[1], d[0])

robot, engine, out = sys.argv[1:4]
mz = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
cfg = ROBOTS[robot]
models = "\n".join(
    f'    <include><name>{n}</name><uri>{u}</uri><pose>{p[0]} {p[1]} {mz if p[2] is None else p[2]} 0 0 0</pose></include>'
    for n, p, u in cfg["models"])
cams = ""
for n, eye, tgt, fov in cfg["cams"]:
    pitch, yaw = lookat(eye, tgt)
    cams += f'''        <sensor name="{n}" type="camera">
          <pose>{eye[0]} {eye[1]} {eye[2]} 0 {pitch:.6f} {yaw:.6f}</pose>
          <update_rate>1</update_rate>
          <camera><horizontal_fov>{fov}</horizontal_fov>
            <image><width>1280</width><height>960</height></image>
            <clip><near>0.05</near><far>50</far></clip>
            <save enabled="true"><path>/out/{n}</path></save>
          </camera>
        </sensor>
'''
t = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "world_template.sdf")).read()
open(out, "w").write(t.replace("@ENGINE@", engine).replace("@MODELS@", models).replace("@CAMERAS@", cams))
