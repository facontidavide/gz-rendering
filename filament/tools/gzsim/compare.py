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
