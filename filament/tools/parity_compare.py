#!/usr/bin/env python3
"""Compare two filament_parity dumps (reference engine vs candidate).

usage: parity_compare.py <ref_dir> <test_dir> [side_by_side.png]

For every output: agreement of the +inf / -inf / finite classification, and
abs error stats where both are finite. Point clouds compare XYZ only.
Wide-angle dumps (PARITY_WIDEANGLE=1) are compared geometrically instead:
object centroid offsets, cutoff circle radius and Project3d, in pixels; the
optional png gets the RGB images side by side (ref left, test right).
"""
import os
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


def srgb_to_linear(v):
    x = v / 255.0
    return np.where(x <= 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4)


def wide_angle(ref_dir, test_dir, ref, tst, png):
    rows = lambda d, f: [l.split() for l in open(f"{d}/{f}")]
    print("wide angle: cutoff circle radius (px, center row)")
    for a, b in zip(rows(ref_dir, "wa_cutoff.txt"),
                    rows(test_dir, "wa_cutoff.txt")):
        ra, rb = float(a[3]), float(b[3])
        d = f"{rb - ra:+.4f}" if ra > 0 and rb > 0 else "n/a"
        print(f"    {a[0]:<14} ref {ra:9.4f} test {rb:9.4f} diff {d}")
    print("wide angle: sphere centroid offset (px), area test/ref, "
          "Project3d max |d| (px, r)")
    geo = {}
    for a, b in zip(rows(ref_dir, "wa_geom.txt"),
                    rows(test_dir, "wa_geom.txt")):
        g = geo.setdefault(a[0], {"c": [], "area": [], "p": [], "r": []})
        va, vb = np.array(a[2:], float), np.array(b[2:], float)
        g["p"].append(np.abs(va[3:5] - vb[3:5]).max())
        g["r"].append(abs(va[5] - vb[5]))
        if va[2] >= 100 and vb[2] >= 100:  # both well visible
            g["c"].append(np.hypot(*(va[:2] - vb[:2])))
            g["area"].append(vb[2] / va[2])
    for lens, g in geo.items():
        c = np.array(g["c"] or [np.nan])
        print(f"    {lens:<14} centroid max {c.max():.3f} mean {c.mean():.3f}"
              f" (n={len(g['c'])}), area {min(g['area']):.3f}.."
              f"{max(g['area']):.3f}, Project3d {max(g['p']):.2e}"
              f" r {max(g['r']):.2e}")
    for name, data in (("ref", ref), ("test", tst)):
        if "wa_l8" in data and "wa_custom_rgb" in data:
            lin = np.round(srgb_to_linear(data["wa_custom_rgb"][..., 0]) * 255)
            d = np.abs(data["wa_l8"][..., 0] - lin)
            print(f"wide angle: {name} L8 vs linear red of RGB: max |d| "
                  f"{d.max():.0f}")
    if png:
        from PIL import Image
        names = [n for n in ref if n.startswith("wa_") and n.endswith("_rgb")]
        pairs = [np.concatenate([ref[n], np.full((ref[n].shape[0], 8, 3),
                 255.0), tst[n]], axis=1) for n in names]
        w = max(p.shape[1] for p in pairs)
        pairs = [np.pad(p, ((0, 8), (0, w - p.shape[1]), (0, 0)),
                        constant_values=255) for p in pairs]
        Image.fromarray(np.concatenate(pairs).astype(np.uint8)).save(png)
        print(f"wrote {png}")


def main(ref_dir, test_dir, png=None):
    ref, tst = load(ref_dir), load(test_dir)
    if os.path.exists(f"{ref_dir}/wa_geom.txt"):
        wide_angle(ref_dir, test_dir, ref, tst, png)
    print(f"{'output':<20}{'n':>8}{'class!=':>9}{'max|d|':>10}"
          f"{'p99|d|':>10}{'p95|d|':>10}{'mean|d|':>10}{'>1cm%':>8}")
    for name in ref:
        if name.startswith("wa_"):
            continue
        a, b = ref[name], tst[name]
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


if __name__ == "__main__":
    main(*sys.argv[1:4])
