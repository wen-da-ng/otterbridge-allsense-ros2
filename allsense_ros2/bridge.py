"""ROS 2 node: connects to an AllSense phone and publishes its sensors as standard messages.

    ALLSENSE_TOKEN=abcd ros2 run allsense_ros2 bridge --ros-args -p url:="http://192.168.1.20:8765/"

The token comes from $ALLSENSE_TOKEN or the file named by token_file, not from a parameter: anyone
on the DDS domain can read parameters (ros2 param get). A token left in url still works; the
bridge then removes it from the parameter and warns.

Topics: /allsense/<device>/sensors/<name> (the names set in the app), plus a combined
/allsense/<device>/imu (accelerometer + gyroscope + orientation) unless imu:=false. If the phone's
mount is set in the app, /allsense/<device>/robot/<name> carry the same sensors in the robot's axes
(frame robot_frame_id, default base_link), and the combined IMU uses them.

Cameras (opt-in): -p cameras:="['back']" publishes /allsense/<device>/camera/<name>/image/compressed
(sensor_msgs/CompressedImage, JPEG), at camera_size (default 640x480) and camera_fps (default 15);
camera_zoom (e.g. 3.0) zooms, switching lenses on multi-lens cameras. camera_codec:=h264 publishes
/allsense/<device>/camera/<name>/video (foxglove_msgs/CompressedVideo, H.264) instead.

Microphones (opt-in): -p microphones:="['mic']" publishes /allsense/<device>/mic/<name>/audio,
audio_stamped and audio_info (audio_common_msgs, as audio_capture does) at mic_rate (default 16000).
"""

from __future__ import annotations

import threading
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import UInt8MultiArray

from . import convert, link

try:
    import allsense
except ImportError as e:  # pragma: no cover - environment problem, explained to the user
    raise SystemExit(
        "The 'allsense' Python package is missing. Install it into the ROS Python, e.g.\n"
        '  pip install "allsense @ git+https://github.com/wen-da-ng/otterbridge-allsense-python"'
    ) from e


class Bridge(Node):
    def __init__(self) -> None:
        super().__init__("allsense_bridge")
        url = self.declare_parameter("url", "").value
        token_param = self.declare_parameter("token", "").value
        token_file = self.declare_parameter("token_file", "").value
        try:
            phone_link = link.resolve(url, token_param, token_file)
        except ValueError as e:
            raise SystemExit(f"allsense_bridge: {e}") from None
        if phone_link.exposed:
            # Parameters are readable by anyone on the DDS domain: keep only the address there.
            self.set_parameters([Parameter("url", Parameter.Type.STRING, phone_link.url),
                                 Parameter("token", Parameter.Type.STRING, "")])
            self.get_logger().warn(f"The pairing token was in a ROS parameter (url or token), which anyone on this ROS "
                                   f"network can read; it's now removed from it. Pass it as ${link.TOKEN_ENV} or in a "
                                   "file named by token_file instead.")
        names: List[str] = list(self.declare_parameter("streams", [""]).value or [])
        names = [n for n in names if n]
        self.rate = float(self.declare_parameter("rate_hz", 50.0).value)
        frame = self.declare_parameter("frame_id", "").value
        self.robot_frame_id = self.declare_parameter("robot_frame_id", "base_link").value
        combined = bool(self.declare_parameter("imu", True).value)
        absolute = self.declare_parameter("imu_orientation", "game").value == "absolute"
        self.use_phone_time = bool(self.declare_parameter("use_phone_time", True).value)
        reliable = bool(self.declare_parameter("reliable", False).value)
        cameras = [c for c in (self.declare_parameter("cameras", [""]).value or []) if c]
        self.camera_size = self.declare_parameter("camera_size", "640x480").value
        self.camera_fps = int(self.declare_parameter("camera_fps", 15).value)
        zoom = float(self.declare_parameter("camera_zoom", 0.0).value)
        self.camera_zoom = zoom if zoom > 0 else None  # e.g. 3.0: the phone switches to its telephoto lens
        self.camera_codec = self.declare_parameter("camera_codec", "jpeg").value
        bitrate = int(self.declare_parameter("camera_bitrate", 0).value)
        self.camera_bitrate = bitrate if bitrate > 0 else None  # H.264 bits/s; the phone's default is 1 000 000
        microphones = [m for m in (self.declare_parameter("microphones", [""]).value or []) if m]
        self.mic_rate = int(self.declare_parameter("mic_rate", 16000).value)

        if not url:
            raise SystemExit("Set the phone's address: --ros-args -p url:=\"http://<phone>:8765/\" and the token in "
                             f"${link.TOKEN_ENV} or token_file (from the app: Home → Connect → ROS 2)")
        if not phone_link.token:
            raise SystemExit(f"No pairing token: set ${link.TOKEN_ENV}, or token_file to a file holding it "
                             "(the token is in the link from the app: Home → Connect → ROS 2)")

        self.phone = allsense.connect(phone_link.url, phone_link.token, name=self.get_name())
        device = self.phone.device
        self.frame_id = frame or f"{device}_sensors"
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE) if reliable else qos_profile_sensor_data
        prefix = f"allsense/{device}"

        # Default: every stream the app shows in Basic mode that has a proper ROS message type.
        if names:
            infos = [self.phone.find(n) for n in names]
        else:
            infos = [s for s in self.phone.streams() if convert.kind(s) in convert.TYPED]

        self._lock = threading.Lock()
        self._pubs: Dict[str, object] = {}
        for info in infos:
            msg_type = convert.message_type(convert.kind(info))
            self._pubs[info.topic] = self.create_publisher(msg_type, f"{prefix}/{info.category}/{info.name}", self.qos)
            self.get_logger().info(f"{info.name} ({info.type}) -> /{prefix}/{info.category}/{info.name} [{msg_type.__name__}]")

        self.combined = None
        # The combined IMU takes all three parts from one frame: the robot's when the phone is mounted.
        self.imu_category = "robot" if any(s.category == "robot" for s in self.phone.streams()) else "sensors"
        if combined:
            orientation = convert.ROTATION_VECTOR if absolute else convert.GAME_ROTATION_VECTOR
            self.combined = convert.CombinedImu(self.robot_frame_id if self.imu_category == "robot" else self.frame_id, orientation)
            self._imu_pub = self.create_publisher(Imu, f"{prefix}/imu", self.qos)
            # The combined IMU needs these three even if they weren't asked for individually.
            want = {convert.ACCELEROMETER, convert.GYROSCOPE, orientation}
            have = {i.sensor_type for i in infos if i.category == self.imu_category}
            extra = [s for s in self.phone.streams() if s.category == self.imu_category and s.sensor_type in want - have]
            infos += extra
            self.get_logger().info(f"combined IMU ({self.imu_category} frame) -> /{prefix}/imu")

        self.subs = [
            self.phone.subscribe(info.topic, rate=self.rate if info.continuous else None, callback=self._make_callback(info))
            for info in infos
        ]
        self._running = True
        if self.camera_codec not in ("jpeg", "h264"):
            raise SystemExit(f"camera_codec is 'jpeg' or 'h264', not {self.camera_codec!r}")
        if self.camera_codec == "h264" and convert.CompressedVideo is None:
            self.get_logger().warn("H.264 needs foxglove_msgs (sudo apt install ros-$ROS_DISTRO-foxglove-msgs); publishing JPEG")
            self.camera_codec = "jpeg"
        for name in cameras:
            info = self.phone.find(name)
            if self.camera_codec == "h264":
                topic, msg_type, loop = f"{prefix}/camera/{info.name}/video", convert.CompressedVideo, self._video_loop
            else:
                topic, msg_type, loop = f"{prefix}/camera/{info.name}/image/compressed", CompressedImage, self._camera_loop
            pub = self.create_publisher(msg_type, topic, self.qos)
            self._start(f"camera {info.name}", loop, info.name, pub)
            self.get_logger().info(f"{info.name} (camera) -> /{topic} [{msg_type.__name__}]")
        for name in microphones:
            info = self.phone.find(name)
            topic = f"{prefix}/mic/{info.name}"
            if convert.AudioData is not None:
                pubs = (self.create_publisher(convert.AudioData, f"{topic}/audio", self.qos),
                        self.create_publisher(convert.AudioDataStamped, f"{topic}/audio_stamped", self.qos))
                # Latched, so subscribers that join later (audio_play) still learn the format.
                latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                info_pub = self.create_publisher(convert.AudioInfo, f"{topic}/audio_info", latched)
                self.get_logger().info(f"{info.name} (mic) -> /{topic}/audio, audio_stamped, audio_info [audio_common_msgs]")
            else:
                pubs, info_pub = (self.create_publisher(UInt8MultiArray, f"{topic}/audio", self.qos),), None
                self.get_logger().warn("No audio_common_msgs (sudo apt install ros-$ROS_DISTRO-audio-common-msgs); "
                                       f"publishing /{topic}/audio as UInt8MultiArray (16-bit PCM)")
            self._start(f"mic {info.name}", self._mic_loop, info.name, pubs, info_pub)
        self.create_timer(5.0, self._report)
        self.get_logger().info(f"Streaming from '{device}' at {self.rate:g} Hz, frame '{self.frame_id}'")

    def _make_callback(self, info):
        pub = self._pubs.get(info.topic)
        max_range_m = None
        if info.sensor_type == convert.PROXIMITY:
            max_range_m = (info.raw.get("max_range") or 5.0) / 100.0  # cm → m
        frame_id = self.robot_frame_id if info.category == "robot" else self.frame_id
        feeds_imu = info.category == self.imu_category

        def on_sample(sample):
            seconds = sample.time if self.use_phone_time else self.get_clock().now().nanoseconds / 1e9
            if pub is not None:
                pub.publish(convert.to_message(convert.kind(info), sample.values, seconds, frame_id, info.fields, max_range_m))
            if self.combined is not None and feeds_imu:
                with self._lock:
                    imu = self.combined.update(info.sensor_type, sample.values, seconds)
                if imu is not None:
                    self._imu_pub.publish(imu)

        return on_sample

    def _start(self, label: str, loop, *args) -> None:
        """Runs one camera or mic stream in a thread; reconnects if it ends (phone gone, camera busy)."""

        def run():
            while self._running:
                try:
                    loop(*args)
                except allsense.AllSenseError as e:
                    self.get_logger().warn(f"{label}: {e}", throttle_duration_sec=30)
                except Exception as e:  # noqa: BLE001 - keep the node alive; report and retry
                    self.get_logger().warn(f"{label} stream ended: {e}", throttle_duration_sec=30)
                time.sleep(2.0)

        threading.Thread(target=run, daemon=True).start()

    def _seconds(self, t_ns) -> float:
        if t_ns is not None and self.use_phone_time:
            return self.phone.clock.to_unix(t_ns)
        return self.get_clock().now().nanoseconds / 1e9

    def _camera_loop(self, name: str, pub) -> None:
        frame_id = f"{self.phone.device}_camera_{name}"
        with self.phone.camera(name, size=self.camera_size, fps=self.camera_fps, zoom=self.camera_zoom) as cam:
            for f in cam:
                if not self._running:
                    return
                pub.publish(convert.compressed_image(f.jpeg, self._seconds(f.t_ns), frame_id))

    def _video_loop(self, name: str, pub) -> None:
        frame_id = f"{self.phone.device}_camera_{name}"
        started = False
        with self.phone.camera(name, size=self.camera_size, fps=self.camera_fps, zoom=self.camera_zoom,
                               codec="h264", bitrate=self.camera_bitrate) as cam:
            for f in cam:
                if not self._running:
                    return
                started = started or bool(f.keyframe)  # decoders need a keyframe (SPS/PPS) first
                if started:
                    pub.publish(convert.compressed_video(f.data, self._seconds(f.t_ns), frame_id))

    def _mic_loop(self, name: str, pubs, info_pub) -> None:
        frame_id = f"{self.phone.device}_mic_{name}"
        with self.phone.microphone(name, rate=self.mic_rate) as mic:
            if info_pub is not None:
                info_pub.publish(convert.audio_info(mic.rate))
            for chunk in mic:
                if not self._running:
                    return
                pcm = convert.pcm16le(chunk.samples)
                if info_pub is None:
                    pubs[0].publish(convert.audio_array(pcm, chunk.rate))
                    continue
                for pub, msg in zip(pubs, convert.audio_data(pcm, self._seconds(chunk.t_ns), frame_id)):
                    pub.publish(msg)

    def _report(self) -> None:
        refused = [s.info.name for s in self.subs if s.status]
        if refused:
            self.get_logger().warn(f"Phone refused: {', '.join(refused)}", throttle_duration_sec=60)
        rtt = self.phone.round_trip_ms
        if rtt is not None:
            self.get_logger().debug(f"clock sync round trip {rtt:.1f} ms")

    def destroy_node(self):
        self._running = False
        self.phone.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, allsense.AllSenseError) as e:
        if isinstance(e, allsense.AllSenseError):
            print(f"allsense_bridge: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
