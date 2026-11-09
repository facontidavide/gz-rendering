#!/usr/bin/env python3
"""usage: compare.py <ref_outdir> <test_outdir>  (outputs of run.sh)"""
import base64
import json
import sys
import numpy as np


def first(path):
    return json.loads(open(path).readline())


def depth(d):
    j = first(f"{d}/depth_image.json")
    return np.frombuffer(base64.b64decode(j["data"]), np.float32)


def scan(d):
    return np.array([float(x) for x in first(f"{d}/scan.json")["ranges"]],
                    np.float32)


def rgb(d):
    return np.frombuffer(base64.b64decode(first(f"{d}/image.json")["data"]),
                         np.uint8).astype(int)


def cls(x):
    return np.where(np.isposinf(x), 1, np.where(np.isneginf(x), -1, 0))


ref, tst = sys.argv[1], sys.argv[2]
for name, f in (("depth", depth), ("lidar", scan)):
    a, b = f(ref), f(tst)
    both = np.isfinite(a) & np.isfinite(b)
    d = np.abs(a - b)[both]
    print(f"{name}: n={a.size} class mismatches={(cls(a) != cls(b)).sum()} "
          f"max|d|={d.max():.6f} p99={np.percentile(d, 99):.6f}")
d = np.abs(rgb(ref) - rgb(tst))
print(f"rgb: mean|d|={d.mean():.2f} p95={np.percentile(d, 95):.0f} levels")


def img(d, name, dtype=np.uint8):
    j = first(f"{d}/{name}.json")
    return np.frombuffer(base64.b64decode(j["data"]), dtype).astype(np.int64)


for name in ("labels_map", "colored_map"):
    a, b = img(ref, name), img(tst, name)
    print(f"seg {name}: n={a.size} differing bytes={(a != b).sum()}")
a, b = img(ref, "thermal", np.uint16), img(tst, "thermal", np.uint16)
d = np.abs(a - b)
print(f"thermal L16: max|d|={d.max()} LSB, differing={(d > 0).mean() * 100:.2f}%"
      f" (0.01 K per LSB)")
d = np.abs(img(ref, "wide") - img(tst, "wide"))
print(f"wide angle rgb: mean|d|={d.mean():.2f} p95={np.percentile(d, 95):.0f}")


def boxes(d, name):
    """(label, numbers...) per box, sorted by label then position"""
    j = first(f"{d}/{name}.json")
    out = []
    for b in j.get("annotatedBox", j.get("annotated_box", [])):
        box = b.get("box", {})
        nums = []
        for key in ("minCorner", "maxCorner", "center", "boxSize"):
            v = box.get(key, {})
            nums += [float(v.get(k, 0.0)) for k in ("x", "y", "z")]
        q = box.get("orientation", {})
        w = float(q.get("w", 1.0))
        sign = -1.0 if w < 0 else 1.0
        nums += [sign * float(q.get(k, 0.0 if k != "w" else 1.0))
                 for k in ("w", "x", "y", "z")]
        out.append((int(b.get("label", 0)), *nums))
    return sorted(out)


for name in ("bbox2d", "bboxfull", "bbox3d"):
    a, b = boxes(ref, name), boxes(tst, name)
    labels = [x[0] for x in a] == [x[0] for x in b]
    dmax = max((abs(x - y) for p, q in zip(a, b) for x, y in zip(p[1:], q[1:])),
               default=0.0)
    print(f"{name}: boxes ref={len(a)} test={len(b)} same labels={labels} "
          f"max|d|={dmax:.4f}")
