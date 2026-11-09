#!/bin/bash
# usage: run.sh <ogre2|filament> <outdir>
# Runs gz-sim headless in a gz-main docker image with the plugin built by
# build_in_container.sh (CB=<its /cb dir>), and grabs one RGB image, depth
# image, lidar scan, segmentation, thermal, bounding box and wide-angle
# messages from gz-transport as JSON. Compare with compare.py.
# IMAGE: gz-main image with libgles-dev/libegl-dev (default below).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
: "${CB:?set CB to the build_in_container.sh output dir}"
IMAGE=${IMAGE:-qpb-gzmain:filament-deps}
eng=$1; out=$(realpath -m "$2")
rm -rf "$out"; mkdir -p "$out"
engname=$eng; [ "$eng" = filament ] && engname=gz-rendering-filament
sed "s#<render_engine>ogre2</render_engine>#<render_engine>$engname</render_engine>#" \
  "$HERE/world.sdf" > "$out/world.sdf"
docker run --rm --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --network none \
  -u "$(id -u):$(id -g)" -e HOME=/tmp -e GZ_IP=127.0.0.1 -e GZ_PARTITION=fil \
  -v "$CB":/cb:ro -v "$out":/out \
  -e GZ_RENDERING_PLUGIN_PATH=/cb/gzr-build/lib \
  -e GZ_RENDERING_FILAMENT_RESOURCE_PATH=/cb/gzr-build/filament/src/media \
  "$IMAGE" bash -c '
    cd /out
    gz sim -s -r --headless-rendering -v 3 /out/world.sdf > /out/sim.log 2>&1 &
    sleep 20
    gz topic -l > /out/topics.txt
    for t in "depth_image" "/scan" "/rgb/image" "/seg/labels_map" \
             "/seg/colored_map" "/thermal" "/bbox2d" "/bboxfull" "/bbox3d" \
             "/wide"; do
      T=$(grep -m1 "$t\$" /out/topics.txt)
      timeout 20 gz topic -e -n 1 --json-output -t "$T" > "/out/$(basename $t).json"
    done
    kill %1; wait' > "$out/log.txt" 2>&1
ls "$out"/*.json
