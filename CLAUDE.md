# CLAUDE.md

`allsense_ros2`, the ROS 2 bridge for AllSense by OtterBridge (ament_python, package at the repo root): `allsense_ros2/bridge.py` (the node: subscribes through the `allsense` Python client, publishes standard messages), `allsense_ros2/convert.py` (pure conversions, unit-tested in `test/`), `launch/`, `config/bridge.yaml`. Public, MIT.

## Working here

- **The user makes every commit and push.** Edit files and run the tests, then hand over: what changed, the test results, and a one-line commit message suggestion. Never run `git commit`, `git push`, tags or releases (bloom included); add no Co-Authored-By line unless asked.
- **Public repository:** never write the user's serials, IPs, Wi-Fi details, home location, tokens or file paths into it. Machine facts go in `CLAUDE.local.md` (git-ignored).

## Test

`tools/test_in_docker.sh`: build, unit tests, and the bridge against the test kit's fake phone (fails without IMU data). In Docker (`ros:humble` or `ros:jazzy`, command in the script's header) or, as CI does, with `REPO`, `PYTHON_SRC` and `TESTKIT_SRC` pointing at checkouts. Without ROS at hand, `python -m pytest test` covers `convert.py` only; always also check that every file parses (`bridge.py` isn't imported by the unit tests).

## How it fits

- Depends on the `allsense` Python package (`otterbridge-allsense-python`), installed with pip into the ROS Python; ROS console scripts don't resolve pip dependencies, so it isn't declared in `setup.py`. The Python client's `mcap_export.py` mirrors `convert.py`: change both together.
- Optional message packages: `foxglove_msgs` (H.264 as `CompressedVideo`) and `audio_common_msgs` (microphones); without them the bridge falls back.
- ROS 2 gotchas: in WSL use Cyclone DDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) and `--no-daemon` for the CLI. ROS setup scripts break under `set -u`. Don't name node attributes `_publishers` and the like (they shadow rclpy internals).
