#!/bin/bash
# usage: run_robots.sh <anymal|franka> <outroot> [engines...]   (default: ogre2 filament)
# Renders the robot with each engine in the gz-sim docker harness (see ../run.sh).
# Results: <outroot>/<robot>_<engine>/<camera>/*.png ; then run compose.py <outroot>.
# Assets are mounted read-only; nothing is copied into the repo.
# Diagnostics: TAG=_x (dir suffix), DOCKER_EXTRA="-e FIL_NOSHADOW=1", WORLD_SED='s/a/b/'.
# Needs CB (plugin build dir), ROS lyrical xacro for franka.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
: "${CB:?set CB to the build_in_container.sh output dir}"
IMAGE=${IMAGE:-qpb-gzmain:filament-deps}
ANYMAL=${ANYMAL:-/home/davide/ws_plotjuggler/DATA/example_assets/example-024-quadruped/anymal_d_simple_description}
FRANKA=${FRANKA:-/home/davide/franka_ws/install/franka_description/share/franka_description}
WAIT=${WAIT:-40}   # seconds of rendering after the cameras are subscribed (1 Hz, one PNG per frame)
robot=$1; root=$(realpath -m "$2"); shift 2; engines=${*:-ogre2 filament}
mkdir -p "$root/work"; W=$root/work
P="python3 $HERE/prep_urdf.py"
case $robot in
anymal)
  PKG=anymal_d_simple_description; ASSET=$ANYMAL; MODELS="anymal"
  # standing pose; prints model z that puts the feet on the ground
  MZ=$($P "$ASSET/urdf/anymal.urdf" "$W/anymal.urdf" --foot-z FOOT --q \
    LF_HAA=0.03,LF_HFE=0.4,LF_KFE=-0.8,RF_HAA=-0.03,RF_HFE=0.4,RF_KFE=-0.8,LH_HAA=0.03,LH_HFE=-0.4,LH_KFE=0.8,RH_HAA=-0.03,RH_HFE=-0.4,RH_KFE=0.8)
  MZ=$(python3 -c "print($MZ + 0.03)") ;;
franka)
  PKG=franka_description; ASSET=$FRANKA; MODELS="franka franka_stl"; MZ=0
  bash -c "source /opt/ros/lyrical/setup.bash; AMENT_PREFIX_PATH=$FRANKA/../..:\$AMENT_PREFIX_PATH \
    xacro $FRANKA/robots/fer/fer.urdf.xacro hand:=true" > "$W/fer.urdf"
  Q=fer_joint1=0.3,fer_joint2=-0.7853981,fer_joint3=0,fer_joint4=-2.3561945,fer_joint5=0,fer_joint6=1.5707963,fer_joint7=0.7853981
  $P "$W/fer.urdf" "$W/franka.urdf" --q $Q --no-inertial
  $P "$W/fer.urdf" "$W/franka_stl.urdf" --q $Q --no-inertial --stl ;;
*) echo "unknown robot $robot"; exit 1 ;;
esac
# xacro's AMENT_PREFIX_PATH above: franka_description lives at <install>/franka_description/share/franka_description
for eng in $engines; do
  out=$root/${robot}_${eng}${TAG}; rm -rf "$out"; mkdir -p "$out"
  engname=$eng; [ "$eng" = filament ] && engname=gz-rendering-filament
  python3 "$HERE/make_world.py" "$robot" "$engname" "$out/world.sdf" "$MZ"
  [ -n "$WORLD_SED" ] && sed -i "$WORLD_SED" "$out/world.sdf"
  cp $(for m in $MODELS; do echo "$W/$m.urdf"; done) "$out/"
  docker run --rm --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all --network none \
    -u "$(id -u):$(id -g)" -e HOME=/tmp -e GZ_IP=127.0.0.1 -e GZ_PARTITION=rob \
    -v "$CB":/cb:ro -v "$out":/out -v "$ASSET":/assets/$PKG:ro \
    -e GZ_RENDERING_PLUGIN_PATH=/cb/gzr-build/lib \
    -e GZ_RENDERING_FILAMENT_RESOURCE_PATH=/cb/gzr-build/filament/src/media \
    $DOCKER_EXTRA -e PKG=$PKG -e MODELS="$MODELS" -e WAIT=$WAIT \
    "$IMAGE" bash -c '
      cd /out
      for m in $MODELS; do
        sed "s#package://$PKG/#/assets/$PKG/#g" $m.urdf > $m.abs.urdf
        gz sdf -p $m.abs.urdf > $m.sdf 2> $m.sdf.err
        sed -i "0,/<model name=.[^>]*>/s//&<static>true<\/static>/" $m.sdf
      done
      gz sim -s -r --headless-rendering -v 3 /out/world.sdf > /out/sim.log 2>&1 &
      sleep 15
      # the Sensors system only renders cameras that have a subscriber
      for T in $(gz topic -l | grep "/sensor/.*/image$"); do gz topic -e -t $T > /dev/null & done
      sleep $WAIT
      kill $(jobs -p); wait' > "$out/log.txt" 2>&1 || true
  ls "$out"/*/ | head -3
done
