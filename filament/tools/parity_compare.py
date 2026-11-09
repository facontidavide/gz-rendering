#!/usr/bin/env python3
"""Compare two filament_parity dumps (reference engine vs candidate).

usage: parity_compare.py <ref_dir> <test_dir>

For every output: agreement of the +inf / -inf / finite classification, and
abs error stats where both are finite. Point clouds compare XYZ only.
"""
import sys
import numpy as np


def load(d):
    out = {}
    for line in open(f"{d}/index.txt"):
        name, w, h, c = line.split()
        a = np.fromfile(f"{d}/{name}.f32", dtype=np.float32)
        out[name] = a.reshape(int(h), int(w), int(c))
    return out


def cls(a):
    return np.where(np.isposinf(a), 1, np.where(np.isneginf(a), -1,
                    np.where(np.isnan(a), 2, 0)))


def main(ref_dir, test_dir):
    ref, tst = load(ref_dir), load(test_dir)
    print(f"{'output':<20}{'n':>8}{'class!=':>9}{'max|d|':>10}"
          f"{'p99|d|':>10}{'p95|d|':>10}{'mean|d|':>10}{'>1cm%':>8}")
    for name in ref:
        a, b = ref[name], tst[name]
        if name.startswith("bbox_"):
            compare_boxes(name, a[..., 0], b[..., 0])
            continue
        if name.endswith("_cloud"):
            a, b = a[..., :3], b[..., :3]
            # a point is finite when all of x, y, z are
            ca = np.where(np.isfinite(a).all(-1), 0, cls(a[..., 0]))
            cb = np.where(np.isfinite(b).all(-1), 0, cls(b[..., 0]))
            fin = (ca == 0) & (cb == 0)
            d = np.linalg.norm(a - b, axis=-1)[fin]
        else:
            a, b = a[..., 0], b[..., 0]
            ca, cb = cls(a), cls(b)
            fin = (ca == 0) & (cb == 0)
            d = np.abs(a - b)[fin]
        mism = int((ca != cb).sum())
        if d.size == 0:
            d = np.zeros(1)
        print(f"{name:<20}{ca.size:>8}{mism:>9}{d.max():>10.4f}"
              f"{np.percentile(d, 99):>10.4f}{np.percentile(d, 95):>10.4f}"
              f"{d.mean():>10.5f}{100.0 * (d > 0.01).mean():>8.2f}")
        if name.startswith("seg_"):
            # label images: report differing pixels; panoptic instance ids
            # may legitimately be numbered differently, so also check that
            # the two images are the same partition (a 1:1 relabelling)
            ra = ref[name].reshape(-1, 3).astype(int)
            rb = tst[name].reshape(-1, 3).astype(int)
            diff = int((ra != rb).any(axis=1).sum())
            ka = ra[:, 0] * 65536 + ra[:, 1] * 256 + ra[:, 2]
            kb = rb[:, 0] * 65536 + rb[:, 1] * 256 + rb[:, 2]
            pairs = set(zip(ka.tolist(), kb.tolist()))
            same = (len(pairs) == len(set(ka.tolist())) ==
                    len(set(kb.tolist())))
            print(f"    differing pixels: {diff}, same partition: {same}, "
                  f"segments: {len(set(ka.tolist()))}")
        if mism:
            pairs = {}
            for x, y in zip(ca.ravel(), cb.ravel()):
                if x != y:
                    pairs[(int(x), int(y))] = pairs.get((int(x), int(y)), 0) + 1
            names = {0: "fin", 1: "+inf", -1: "-inf", 2: "nan"}
            print("    class ref->test: " + ", ".join(
                f"{names[k[0]]}->{names[k[1]]}:{v}" for k, v in
                sorted(pairs.items())))


def compare_boxes(name, a, b):
    """Rows: center xyz, size xyz, quaternion wxyz, label. Boxes are
    matched in output order (both engines sort by model name)."""
    print(f"{name:<20} boxes ref={len(a)} test={len(b)}", end="")
    if len(a) != len(b) or len(a) == 0:
        print("  COUNT/EMPTY MISMATCH")
        return
    labels = int((a[:, 10] != b[:, 10]).sum())
    dc = np.abs(a[:, 0:3] - b[:, 0:3]).max()
    ds = np.abs(a[:, 3:6] - b[:, 3:6]).max()
    # rotation: angle between quaternions (sign independent)
    dot = np.clip(np.abs((a[:, 6:10] * b[:, 6:10]).sum(1)), 0, 1)
    dang = np.degrees(2 * np.arccos(dot)).max()
    print(f"  label!={labels}  max|dcenter|={dc:.4f}  max|dsize|={ds:.4f}"
          f"  max rot={dang:.3f}deg")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
