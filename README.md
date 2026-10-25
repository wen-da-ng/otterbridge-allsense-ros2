# allsense_ros2

Publishes the sensors of an Android phone running **OtterBridge AllSense** as standard ROS 2 messages. The phone doesn't run ROS: this node runs on your robot's computer and talks to the phone over Wi-Fi or USB with the [`allsense`](https://github.com/wen-da-ng/otterbridge-allsense-python) Python client.

Tested with ROS 2 Humble (Ubuntu 22.04). It uses only `rclpy` and standard messages, so other distributions should work.

## Install

```bash
# 1. The Python client, into the same Python ROS uses
pip install "allsense @ git+https://github.com/wen-da-ng/otterbridge-allsense-python"   # or: pip install allsense (once on PyPI)

# 2. This package, in a colcon workspace
cd ~/ros2_ws/src && git clone https://github.com/wen-da-ng/otterbridge-allsense-ros2 allsense_ros2
cd ~/ros2_ws && colcon build --packages-select allsense_ros2 && source install/setup.bash
```

## Run

In the app, tap **Start streaming** (Home) and copy the link from **Connect → ROS 2**:

```bash
ros2 run allsense_ros2 bridge --ros-args -p url:="http://192.168.1.20:8765/?token=abcd"
# or, with the parameter file:
ros2 launch allsense_ros2 bridge.launch.py url:="http://192.168.1.20:8765/?token=abcd"
```

## Topics

Names follow the app: `/allsense/<device>/sensors/<name>`, using the names you set with **Rename**. Add `--ros-args -r __ns:=/robot1` to put them under a namespace.

| Android sensor | Message | Units |
|---|---|---|
| accelerometer, gravity, linear acceleration | `sensor_msgs/Imu` (only `linear_acceleration`) | m/s² |
| gyroscope | `sensor_msgs/Imu` (only `angular_velocity`) | rad/s |
| rotation vector, game rotation vector | `sensor_msgs/Imu` (only `orientation`) | quaternion |
| magnetic field | `sensor_msgs/MagneticField` | T (converted from µT) |
| pressure | `sensor_msgs/FluidPressure` | Pa (from hPa) |
| light | `sensor_msgs/Illuminance` | lx |
| proximity | `sensor_msgs/Range` | m (from cm) |
| temperature / humidity | `sensor_msgs/Temperature` / `RelativeHumidity` | °C / 0–1 |
| anything else | `std_msgs/Float64MultiArray` (labels in `layout.dim[0].label`) | as reported |

GNSS position is `sensor_msgs/NavSatFix` and the battery `sensor_msgs/BatteryState`. A microphone's loudness (`<mic>_level`: rms and peak in dBFS) has no standard message: list it in `streams` to get a labelled `Float64MultiArray`.

Fields an `Imu` message doesn't carry are marked "not provided" (covariance[0] = −1), as ROS expects.

**`/allsense/<device>/imu`** combines accelerometer, gyroscope and orientation into one `sensor_msgs/Imu` (one message per accelerometer sample), the form `robot_localization` and most IMU consumers want.

## Cameras and microphones

Opt-in, by name (as in the app):

| Parameter | Topic | Message |
|---|---|---|
| `cameras:="['back']"` | `…/camera/<name>/image/compressed` | `sensor_msgs/CompressedImage` (JPEG; RViz, image_transport, Foxglove) |
| `cameras:="['back']"` `camera_codec:=h264` | `…/camera/<name>/video` | `foxglove_msgs/CompressedVideo` (H.264, Annex B; about a tenth of the bandwidth; Foxglove plays it) |
| `microphones:="['mic']"` | `…/mic/<name>/audio`, `audio_stamped`, `audio_info` (latched) | `audio_common_msgs` AudioData / AudioDataStamped / AudioInfo: mono S16LE PCM, chunks of 0.1 s, the same topics as `audio_capture` |

The optional message packages: `sudo apt install ros-$ROS_DISTRO-foxglove-msgs ros-$ROS_DISTRO-audio-common-msgs`. Without them the bridge says so and falls back: JPEG instead of H.264, and audio as `std_msgs/UInt8MultiArray` (16-bit little-endian PCM; the format is in `layout.dim[0].label`, e.g. `pcm_s16le,16000Hz,mono`).

H.264 publishing starts at the first keyframe (it carries SPS/PPS), so a viewer can always decode from the first message. Video and audio are stamped with the phone's capture time (clock-synced) unless `use_phone_time:=false`.

Listen to the phone with audio_common's player (`sudo apt install ros-$ROS_DISTRO-audio-play`); it takes the format as parameters, not from `audio_info`:

```bash
ros2 launch audio_play play.launch.xml format:=wave sample_rate:=16000 ns:=/allsense/<device>/mic/mic
```

## Parameters

See [`config/bridge.yaml`](config/bridge.yaml).

| Parameter | Default | |
|---|---|---|
| `url` | (required) | Link from the app; contains the token |
| `streams` | Basic streams with a ROS message type | Names to publish |
| `rate_hz` | 50 | Continuous sensors; event sensors send every change |
| `frame_id` | `<device>_sensors` | |
| `robot_frame_id` | `base_link` | Frame of the robot-frame topics (app: Robot → Mount on the robot) and of `/imu` when they exist |
| `imu` | true | Publish the combined IMU |
| `imu_orientation` | `game` | `game` (no compass; steady near motors, yaw drifts slowly) or `absolute` (compass; heading from north) |
| `use_phone_time` | true | Stamp with the sensor's clock-synced time, not the arrival time |
| `reliable` | false | QoS is best effort (sensor data); true for RELIABLE subscribers |
| `cameras` | none | Cameras to publish |
| `camera_size`, `camera_fps`, `camera_zoom` | 640x480, 15, 0 (none) | The first viewer of a camera sets these on the phone |
| `camera_codec` | `jpeg` | `jpeg` or `h264` |
| `camera_bitrate` | 0 (phone default, 1 Mbit/s) | H.264 bits/s |
| `microphones` | none | Microphones to publish |
| `mic_rate` | 16000 | Hz (the phone offers 8000, 16000, 48000…) |

## Frames and mounting

**Easiest: set the mount in the app** (Robot → Mount on the robot). The bridge then also publishes `/allsense/<device>/robot/<name>` (accelerometer, gyroscope, magnetometer, gravity, rotation vectors) already in the robot's axes, in frame `robot_frame_id` (default `base_link`), and the combined `/allsense/<device>/imu` uses them. No transform needed.

Otherwise, messages use the phone's sensor frame: **x** to the right of the screen, **y** up the screen, **z** out of the screen. Orientations are relative to East-North-Up, ROS's world convention. Publish a static transform from your robot's frame to `frame_id`. For example, for a phone lying flat, screen up, top of the phone pointing forward:

```bash
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id phone_sensors --yaw -1.5708
```

## WSL notes

- With WSL's **mirrored** networking (`networkingMode=Mirrored` in `.wslconfig`), ROS 2's default Fast DDS can discover nodes but deliver no messages between processes; topics show up in `ros2 topic list` but nothing arrives. Switching to Cyclone DDS fixes it:
  ```bash
  sudo apt install -y ros-humble-rmw-cyclonedds-cpp
  echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc   # then open a new terminal
  ```
- If `ros2 topic list` hangs or times out, the ROS 2 CLI daemon is stuck: `ros2 daemon stop`, or add `--no-daemon`.
- The phone is reached from WSL at its normal Wi-Fi address (e.g. `192.168.1.20`); mDNS discovery may not work across WSL.

## Testing without a phone

`tools/test_in_docker.sh` builds the package in the `ros:humble` Docker image and checks it end to end against the fake phone from [`allsense-testkit`](https://github.com/wen-da-ng/otterbridge-allsense-protocol). It needs this repository, the Python client and the protocol repository side by side:

```bash
docker run --rm -v "$PWD:/repo:ro" -v "$PWD/../otterbridge-allsense-python:/python:ro" \
  -v "$PWD/../otterbridge-allsense-protocol:/testkit:ro" ros:humble bash /repo/tools/test_in_docker.sh
```

## License

MIT. The AllSense app is a free download from [otterbridge-allsense](https://github.com/wen-da-ng/otterbridge-allsense); docs at [docs.allsense.otterbridge.org](https://docs.allsense.otterbridge.org/get-started/ros2/).
