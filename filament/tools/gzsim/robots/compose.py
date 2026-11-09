#!/usr/bin/env python3
"""compose.py ROOT [OUTDIR]  -- ogre2 | filament side-by-side PNG per robot/camera.

ROOT holds <robot>_ogre2/<cam>/*.png and <robot>_filament/<cam>/*.png from
run_robots.sh. Uses the last complete frame of each camera. Prints the mean
absolute RGB difference (0-255) per pair.
"""
import sys, glob, os, re
import numpy as np
from PIL import Image, ImageDraw, ImageFont

root = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else root

def last_frame(d):
    fs = sorted(glob.glob(os.path.join(d, "*.png")), key=lambda f: int(re.search(r"_(\d+)\.png$", f).group(1)))
    for f in reversed(fs[:-1] or fs):  # skip the newest: may still be written when the sim is killed
        try:
            return Image.open(f).convert("RGB")
        except Exception:
            continue

def label(im, text):
    d = ImageDraw.Draw(im)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 36)
    except OSError:
        font = ImageFont.load_default()
    w = d.textlength(text, font=font)
    d.rectangle([0, 0, w + 24, 56], fill=(0, 0, 0))
    d.text((12, 8), text, fill=(255, 255, 255), font=font)

os.makedirs(out, exist_ok=True)
for od in sorted(glob.glob(os.path.join(root, "*_ogre2"))):
    robot = os.path.basename(od)[:-len("_ogre2")]
    fd = os.path.join(root, robot + "_filament")
    for cd in sorted(glob.glob(os.path.join(od, "*/"))):
        cam = os.path.basename(cd.rstrip("/"))
        o, f = last_frame(cd), last_frame(os.path.join(fd, cam))
        if o is None or f is None:
            print(f"{robot}/{cam}: missing frame (ogre2={o is not None}, filament={f is not None})"); continue
        d = np.abs(np.asarray(o, np.int16) - np.asarray(f, np.int16)).mean()
        o, f = o.copy(), f.copy()
        label(o, f"{robot}/{cam}  ogre2"); label(f, f"{robot}/{cam}  filament  (mean |dRGB| = {d:.2f})")
        sheet = Image.new("RGB", (o.width + f.width + 8, o.height), (255, 255, 255))
        sheet.paste(o, (0, 0)); sheet.paste(f, (o.width + 8, 0))
        p = os.path.join(out, f"{robot}_{cam}.png"); sheet.save(p)
        print(f"{robot}/{cam}: mean abs RGB diff = {d:.2f}  -> {p}")
