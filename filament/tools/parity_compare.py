#!/usr/bin/env python3
"""Compare two filament_parity dumps (reference engine vs candidate).

usage: parity_compare.py <ref_dir> <test_dir> [side_by_side.png]

For every output: agreement of the +inf / -inf / finite classification, and
abs error stats where both are finite. Point clouds compare XYZ only.
Noise dumps (PARITY_NOISE=1) are compared as noise distributions.
Wide-angle dumps (PARITY_WIDEANGLE=1) are compared geometrically instead:
object centroid offsets, cutoff circle radius and Project3d, in pixels; the
optional png gets the RGB images side by side (ref left, test right).
"""
import os
import re
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


def corr(a, b):
    a, b = a.ravel() - a.mean(), b.ravel() - b.mean()
    den = np.sqrt((a * a).sum() * (b * b).sum())
    return float((a * b).sum() / den) if den > 0 else float("nan")


def independence(d):
    """d: frames x H x W. Correlation with the right / lower neighbour and
    with the next frame, fraction of pixels differing frame to frame."""
    return (corr(d[:, :, :-1], d[:, :, 1:]), corr(d[:, :-1], d[:, 1:]),
            corr(d[:-1], d[1:]), float((d[0] != d[1]).mean()))


def noise(ref_dir, test_dir, ref, tst):
    """PARITY_NOISE=1 dumps: noise distributions side by side."""
    cfg = {l.split()[0]: tuple(map(float, l.split()[1:]))
           for l in open(f"{ref_dir}/noise_cfg.txt")}
    rng = np.random.default_rng(0)
    frames = lambda d, p: np.stack([d[k] for k in sorted(d) if
                                    re.fullmatch(re.escape(p) + r"_f\d+", k)])
    print("noise: (noisy - clean) per pixel and channel; lin = linear space"
          " (where ogre2 adds it); model = Monte Carlo of ogre2's shader")
    print("    independence: corr with right / lower neighbour, next frame;"
          " fraction of pixels changing frame to frame")
    for label, data in (("ref", ref), ("test", tst)):
        d = np.abs(data["noise_rgb_zero_f0"] - data["noise_rgb_zero_clean"])
        print(f"  rgb_zero (mean 0, stddev 0, lit scene) {label}: max "
              f"|noisy - clean| {d.max():.0f}")
    for name in ("rgb_wide", "rgb_biased", "wa_face", "wa_final"):
        mean, std = cfg[name]
        z = rng.normal(mean, std, 200000)
        n = np.sign(z) * np.abs(z) ** 2.1
        print(f"  {name} (mean {mean}, stddev {std})")
        for label, data in (("ref", ref), ("test", tst)):
            clean = data[f"noise_{name.split('_')[0]}_clean"
                         if name.startswith("wa") else f"noise_{name}_clean"]
            noisy = frames(data, f"noise_{name}")
            inside = clean[..., 0] > 0  # wide angle: inside the lens circle
            for ch in range(3):
                c, v = clean[..., ch], noisy[..., ch]
                d8 = (v - c)[:, inside]
                dl = (srgb_to_linear(v) - srgb_to_linear(c))[:, inside]
                c0 = float(np.median(c[inside]))
                lc = float(srgb_to_linear(np.array(c0)))
                ml = np.clip(lc + n, 0, 1) - lc
                ind = independence(v - c)
                print(f"    {label:<4} ch{ch} clean {c0:5.1f} | bytes mean "
                      f"{d8.mean():+7.3f} std {d8.std():6.3f} @0 "
                      f"{(v[:, inside] == 0).mean():.3f} @255 "
                      f"{(v[:, inside] == 255).mean():.3f} | lin mean "
                      f"{dl.mean():+.4f} std {dl.std():.4f} (model "
                      f"{ml.mean():+.4f} {ml.std():.4f}) | indep "
                      f"{ind[0]:+.3f} {ind[1]:+.3f} {ind[2]:+.3f} "
                      f"{ind[3]:.3f}")

    names = {0: "fin", 1: "+inf", -1: "-inf", 2: "nan"}
    for name in ("depth", "depth_near", "depth_far"):
        mean, std = cfg[name]
        print(f"  {name} (mean {mean} m, stddev {std} m)")
        for label, data in (("ref", ref), ("test", tst)):
            c = data["noise_depth_clean"][..., 0]
            v = frames(data, f"noise_{name}")[..., 0]
            fin = np.isfinite(c) & np.isfinite(v)
            with np.errstate(invalid="ignore"):
                diff = v - c
            d = diff[fin]
            ca, cb = np.broadcast_to(cls(c), v.shape), cls(v)
            trans = {k: int(((ca == k[0]) & (cb == k[1])).sum())
                     for k in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, -1),
                               (-1, 1))}
            ind = independence(np.where(fin, diff - d.mean(), 0))
            # finite points beyond far / nearer than near
            cv = frames(data, f"noise_cloud_{name}")[..., :3]
            rng_ = np.linalg.norm(cv, axis=-1)
            print(f"    {label:<4} depth mean {d.mean():+.5f} std "
                  f"{d.std():.5f} n {d.size} | finite range > far "
                  f"{int((np.isfinite(rng_) & (rng_ > 10)).sum())}, x < "
                  f"near {int((np.isfinite(v) & (v < 0.1)).sum())} | "
                  + (", ".join(f"{names[k[0]]}->{names[k[1]]}:{n}"
                               for k, n in trans.items() if n) or
                     "no class change") +
                  f" | indep {ind[0]:+.3f} {ind[1]:+.3f} {ind[2]:+.3f} "
                  f"{ind[3]:.3f}")
            if name != "depth":
                continue
            cc = data["noise_cloud_depth_clean"]
            cv = frames(data, "noise_cloud_depth")
            fin = (np.isfinite(cc[..., :3]).all(-1) &
                   np.isfinite(cv[..., :3]).all(-1))
            dxyz = (cv[..., :3] - np.where(np.isfinite(cc[..., :3]),
                                           cc[..., :3], 0))[fin]
            print(f"    {label:<4} cloud dx {dxyz[:, 0].mean():+.5f}/"
                  f"{dxyz[:, 0].std():.5f} max|dx-dy| "
                  f"{np.abs(dxyz[:, 0] - dxyz[:, 1]).max():.2e} max|dx-dz| "
                  f"{np.abs(dxyz[:, 0] - dxyz[:, 2]).max():.2e}")
            pc = cc[..., 3].copy().view(np.uint32)
            pv = cv[..., 3].copy().view(np.uint32)
            for ch, s in (("r", 24), ("g", 16), ("b", 8), ("a", 0)):
                a = ((pc >> s) & 0xFF).astype(float)
                b = ((pv >> s) & 0xFF).astype(float)
                print(f"    {label:<4} cloud {ch}: clean mean {a.mean():6.1f}"
                      f" diff mean {(b - a).mean():+.3f} std "
                      f"{(b - a).std():.3f} @0 {(b == 0).mean():.3f} @255 "
                      f"{(b == 255).mean():.3f}")

    print("  thermal (pass attached, ogre2 never applies it)")
    for label, data in (("ref", ref), ("test", tst)):
        c = data["noise_thermal_clean"]
        v = frames(data, "noise_thermal")
        print(f"    {label:<4} max |noisy - clean| {np.abs(v - c).max():.1f}")


def main(ref_dir, test_dir, png=None):
    ref, tst = load(ref_dir), load(test_dir)
    if os.path.exists(f"{ref_dir}/noise_cfg.txt"):
        noise(ref_dir, test_dir, ref, tst)
        return
    if os.path.exists(f"{ref_dir}/wa_geom.txt"):
        wide_angle(ref_dir, test_dir, ref, tst, png)
    print(f"{'output':<20}{'n':>8}{'class!=':>9}{'max|d|':>10}"
          f"{'p99|d|':>10}{'p95|d|':>10}{'mean|d|':>10}{'>1cm%':>8}")
    for name in ref:
        if name.startswith("wa_"):
            continue
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
    main(*sys.argv[1:4])
