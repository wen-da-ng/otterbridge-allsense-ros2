#!/usr/bin/env bash
# Builds allsense_ros2 in an official ROS 2 image and checks it end to end against the fake phone from
# allsense-testkit, so no phone or local ROS install is needed. Expects this repo at /repo,
# otterbridge-allsense-python at /python and otterbridge-allsense-protocol at /testkit (read-only is fine;
# REPO, PYTHON_SRC, TESTKIT_SRC and WS override the paths, as in CI; PORT the fake phone's port):
#
#   docker run --rm -v "$PWD:/repo:ro" -v "$PWD/../otterbridge-allsense-python:/python:ro" \
#     -v "$PWD/../otterbridge-allsense-protocol:/testkit:ro" ros:humble bash /repo/tools/test_in_docker.sh
#
# Works on Humble and Jazzy (Ubuntu 24.04 needs pip's --break-system-packages, added automatically).
# Fails if the bridge publishes no IMU data with the fake phone's values.
set -eo pipefail   # not -u: ROS setup scripts read unset variables
REPO=${REPO:-/repo} PYTHON_SRC=${PYTHON_SRC:-/python} TESTKIT_SRC=${TESTKIT_SRC:-/testkit} WS=${WS:-/ws} PORT=${PORT:-8765}
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
export PYTHONDONTWRITEBYTECODE=1

echo "== installing the allsense Python client and the test kit"
apt-get update -qq >/dev/null && apt-get install -y -qq python3-pip python3-pytest >/dev/null
# Optional message packages: the bridge falls back without them, and a distro may not have them.
for p in foxglove-msgs audio-common-msgs; do apt-get install -y -qq "ros-$ROS_DISTRO-$p" >/dev/null 2>&1 || echo "   (no ros-$ROS_DISTRO-$p; testing the fallback)"; done
PIP="pip3 install -q"; pip3 install --help | grep -q break-system-packages && PIP="$PIP --break-system-packages"
rm -rf /tmp/testkit /tmp/allsense-python   # pip builds in place: copy the read-only mounts
cp -r "$TESTKIT_SRC" /tmp/testkit && cp -r "$PYTHON_SRC" /tmp/allsense-python
$PIP /tmp/testkit /tmp/allsense-python

echo "== colcon build"
rm -rf "$WS" && mkdir -p "$WS/src" && cp -r "$REPO" "$WS/src/allsense_ros2"
cd "$WS" && colcon build --packages-select allsense_ros2 --event-handlers console_direct- >/dev/null
source install/setup.bash

echo "== unit tests"
python3 -m pytest -q -p no:cacheprovider src/allsense_ros2/test

echo "== bridge against the fake phone"
allsense-fake-phone --port "$PORT" &
sleep 1
ALLSENSE_TOKEN=testtoken ros2 run allsense_ros2 bridge --ros-args -p url:="http://127.0.0.1:$PORT/" -p rate_hz:=50.0 \
  -p "cameras:=['back']" -p camera_codec:=h264 -p "microphones:=['mic']" &
sleep 5
echo "-- topics"
ros2 topic list --no-daemon | grep allsense || true
for t in imu sensors/magnetic_field sensors/pressure sensors/proximity sensors/accelerometer mic/mic/audio_info; do
  echo "-- /allsense/rover/$t"
  timeout 10 ros2 topic echo --no-daemon --once --qos-reliability best_effort "/allsense/rover/$t" | head -40 || true
done
echo "-- check: a token in url is removed from the parameter"
ros2 run allsense_ros2 bridge --ros-args -r __node:=allsense_bridge_url -p url:="http://127.0.0.1:$PORT/?token=testtoken" -p imu:=false \
  -p "streams:=['light']" &
sleep 4
ros2 param get --no-daemon /allsense_bridge_url url | tee /tmp/url.txt
kill %3 2>/dev/null || true
grep -q "127.0.0.1" /tmp/url.txt && ! grep -q testtoken /tmp/url.txt || { echo "FAIL: the token is still in the url parameter"; exit 1; }
echo "-- check: IMU data with the fake phone's gravity"
timeout 10 ros2 topic echo --no-daemon --once --qos-reliability best_effort /allsense/rover/imu > /tmp/imu.txt || true
kill %1 %2 2>/dev/null || true
grep -q "z: 9.81" /tmp/imu.txt || { echo "FAIL: no IMU data from the bridge"; exit 1; }
echo "== done"
