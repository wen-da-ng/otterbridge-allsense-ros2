"""Phone samples (Android sensors, GNSS, battery) → standard ROS 2 messages.

Pure functions (no node, no network) so they can be unit-tested. Values arrive in Android units and
are converted to SI as ROS expects (µT → T, hPa → Pa, cm → m, % → 0..1).

Frames: messages are in the phone's sensor frame (Android: x right, y up along the screen, z out of
the screen). Publish a static transform from your robot's frame to ``frame_id`` for where the phone
is mounted. Rotation vectors are relative to East-North-Up, which matches ROS's ENU world convention.
"""

from __future__ import annotations

import math
import sys
from typing import Optional, Sequence

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import (BatteryState, CompressedImage, FluidPressure, Illuminance, Imu, MagneticField, NavSatFix, NavSatStatus, Range,
                             RelativeHumidity, Temperature)
from std_msgs.msg import Float64MultiArray, Header, MultiArrayDimension, UInt8MultiArray

# Optional message packages (rosdep installs them; the bridge falls back without them).
try:  # ros-<distro>-foxglove-msgs: H.264 video that Foxglove plays
    from foxglove_msgs.msg import CompressedVideo
except ImportError:  # pragma: no cover - depends on the install
    CompressedVideo = None
try:  # ros-<distro>-audio-common-msgs: the usual ROS audio messages (audio_capture, audio_play)
    from audio_common_msgs.msg import AudioData, AudioDataStamped, AudioInfo
except ImportError:  # pragma: no cover - depends on the install
    AudioData = AudioDataStamped = AudioInfo = None

# android.hardware.Sensor.TYPE_*
ACCELEROMETER = 1
MAGNETIC_FIELD = 2
GYROSCOPE = 4
LIGHT = 5
PRESSURE = 6
PROXIMITY = 8
GRAVITY = 9
LINEAR_ACCELERATION = 10
ROTATION_VECTOR = 11
RELATIVE_HUMIDITY = 12
AMBIENT_TEMPERATURE = 13
MAGNETIC_FIELD_UNCALIBRATED = 14
GAME_ROTATION_VECTOR = 15
GYROSCOPE_UNCALIBRATED = 16
GEOMAGNETIC_ROTATION_VECTOR = 20
ACCELEROMETER_UNCALIBRATED = 35

ACCELERATION_TYPES = {ACCELEROMETER, GRAVITY, LINEAR_ACCELERATION, ACCELEROMETER_UNCALIBRATED}
GYRO_TYPES = {GYROSCOPE, GYROSCOPE_UNCALIBRATED}
MAGNETIC_TYPES = {MAGNETIC_FIELD, MAGNETIC_FIELD_UNCALIBRATED}
ROTATION_TYPES = {ROTATION_VECTOR, GAME_ROTATION_VECTOR, GEOMAGNETIC_ROTATION_VECTOR}

# Streams that aren't Android sensors are keyed by their catalog "stream_type".
GNSS_POSITION = "allsense.gnss.position"
BATTERY = "allsense.battery"

#: Types with a dedicated ROS message; everything else becomes Float64MultiArray.
TYPED = ACCELERATION_TYPES | GYRO_TYPES | MAGNETIC_TYPES | ROTATION_TYPES | {
    LIGHT, PRESSURE, PROXIMITY, RELATIVE_HUMIDITY, AMBIENT_TEMPERATURE, GNSS_POSITION, BATTERY,
}


def kind(info):
    """What a stream is, for this module: Android's sensor type number for sensors, the
    ``stream_type`` string for the phone's other streams (GNSS, battery…)."""
    return info.sensor_type if info.sensor_type is not None else (getattr(info, "stream_type", "") or None)

#: In ROS, a covariance whose first element is -1 means "this field isn't provided".
NOT_PROVIDED = [-1.0] + [0.0] * 8


def message_type(sensor_type: Optional[int]):
    """The ROS message class used for an Android sensor type."""
    if sensor_type in ACCELERATION_TYPES | GYRO_TYPES | ROTATION_TYPES:
        return Imu
    return {
        MAGNETIC_FIELD: MagneticField,
        MAGNETIC_FIELD_UNCALIBRATED: MagneticField,
        LIGHT: Illuminance,
        PRESSURE: FluidPressure,
        PROXIMITY: Range,
        RELATIVE_HUMIDITY: RelativeHumidity,
        AMBIENT_TEMPERATURE: Temperature,
        GNSS_POSITION: NavSatFix,
        BATTERY: BatteryState,
    }.get(sensor_type, Float64MultiArray)


def stamp(seconds: float) -> Time:
    sec = int(math.floor(seconds))
    return Time(sec=sec, nanosec=min(int(round((seconds - sec) * 1e9)), 999_999_999))


def header(seconds: float, frame_id: str) -> Header:
    return Header(stamp=stamp(seconds), frame_id=frame_id)


def _f(v) -> float:
    return float("nan") if v is None else float(v)


def vector3(values: Sequence[Optional[float]], scale: float = 1.0) -> Vector3:
    x, y, z = (list(values) + [None] * 3)[:3]
    return Vector3(x=_f(x) * scale, y=_f(y) * scale, z=_f(z) * scale)


def quaternion(values: Sequence[Optional[float]]) -> Quaternion:
    """Android rotation vector (x·sin(θ/2), y·sin(θ/2), z·sin(θ/2)[, cos(θ/2)]) → unit quaternion."""
    x, y, z = (_f(v) for v in (list(values) + [None] * 3)[:3])
    if len(values) > 3 and values[3] is not None:
        w = float(values[3])
    else:  # very old phones omit w
        w = math.sqrt(max(0.0, 1.0 - x * x - y * y - z * z))
    norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    return Quaternion(x=x / norm, y=y / norm, z=z / norm, w=w / norm)


def empty_imu(seconds: float, frame_id: str) -> Imu:
    msg = Imu(header=header(seconds, frame_id))
    msg.orientation_covariance = list(NOT_PROVIDED)
    msg.angular_velocity_covariance = list(NOT_PROVIDED)
    msg.linear_acceleration_covariance = list(NOT_PROVIDED)
    return msg


def to_message(sensor_type: Optional[int], values: Sequence[Optional[float]], seconds: float, frame_id: str,
               fields: Sequence[str] = (), max_range_m: Optional[float] = None):
    """Converts one sample. Fields that a message has but the sensor doesn't measure are marked
    "not provided" (covariance[0] = -1 for Imu) rather than left as zeros."""
    if sensor_type in ACCELERATION_TYPES:
        msg = empty_imu(seconds, frame_id)
        msg.linear_acceleration = vector3(values)
        msg.linear_acceleration_covariance = [0.0] * 9  # provided, covariance unknown
        return msg
    if sensor_type in GYRO_TYPES:
        msg = empty_imu(seconds, frame_id)
        msg.angular_velocity = vector3(values)
        msg.angular_velocity_covariance = [0.0] * 9
        return msg
    if sensor_type in ROTATION_TYPES:
        msg = empty_imu(seconds, frame_id)
        msg.orientation = quaternion(values)
        msg.orientation_covariance = [0.0] * 9
        # Rotation vector's 5th value is the heading accuracy (rad); use it as yaw variance.
        if sensor_type == ROTATION_VECTOR and len(values) > 4 and values[4] is not None and values[4] >= 0:
            msg.orientation_covariance[8] = float(values[4]) ** 2
        return msg
    if sensor_type in MAGNETIC_TYPES:
        return MagneticField(header=header(seconds, frame_id), magnetic_field=vector3(values, 1e-6))  # µT → T
    if sensor_type == PRESSURE:
        return FluidPressure(header=header(seconds, frame_id), fluid_pressure=_f(values[0]) * 100.0)  # hPa → Pa
    if sensor_type == LIGHT:
        return Illuminance(header=header(seconds, frame_id), illuminance=_f(values[0]))
    if sensor_type == AMBIENT_TEMPERATURE:
        return Temperature(header=header(seconds, frame_id), temperature=_f(values[0]))
    if sensor_type == RELATIVE_HUMIDITY:
        return RelativeHumidity(header=header(seconds, frame_id), relative_humidity=_f(values[0]) / 100.0)
    if sensor_type == PROXIMITY:
        # Many phones only report "near" (0) or "far" (= max range).
        max_range = max_range_m if max_range_m and max_range_m > 0 else 0.05
        return Range(header=header(seconds, frame_id), radiation_type=Range.INFRARED,
                     field_of_view=0.5, min_range=0.0, max_range=max_range, range=_f(values[0]) / 100.0)
    if sensor_type == GNSS_POSITION:
        return nav_sat_fix(values, seconds, frame_id)
    if sensor_type == BATTERY:
        return battery_state(values, seconds, frame_id)
    # Anything else: the raw values, labelled. (No header: Float64MultiArray has no timestamp.)
    msg = Float64MultiArray()
    msg.layout.dim = [MultiArrayDimension(label=",".join(fields) or "values", size=len(values), stride=len(values))]
    msg.data = [_f(v) for v in values]
    return msg


def _opt(values, i) -> Optional[float]:
    v = values[i] if i < len(values) else None
    return float(v) if v is not None and math.isfinite(float(v)) else None


def nav_sat_fix(values: Sequence[Optional[float]], seconds: float, frame_id: str) -> NavSatFix:
    """GNSS position: latitude, longitude (°), altitude (m), accuracy (m, 68%), speed, bearing,
    vertical accuracy (m). Accuracies become an approximated diagonal covariance."""
    lat, lon, alt, acc, vacc = (_opt(values, i) for i in (0, 1, 2, 3, 6))
    msg = NavSatFix(header=header(seconds, frame_id))
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.status.status = NavSatStatus.STATUS_FIX if lat is not None and lon is not None else NavSatStatus.STATUS_NO_FIX
    msg.latitude = lat if lat is not None else math.nan
    msg.longitude = lon if lon is not None else math.nan
    msg.altitude = alt if alt is not None else math.nan
    if acc is not None:
        v = vacc if vacc is not None else acc * 1.5  # phones often omit vertical accuracy; it's usually worse
        msg.position_covariance = [acc ** 2, 0.0, 0.0, 0.0, acc ** 2, 0.0, 0.0, 0.0, v ** 2]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    else:
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    return msg


def battery_state(values: Sequence[Optional[float]], seconds: float, frame_id: str) -> BatteryState:
    """Battery: level (%), voltage (V), temperature (°C), current (mA), charging (0/1)."""
    level, volts, temp, ma, charging = (_opt(values, i) for i in range(5))
    msg = BatteryState(header=header(seconds, frame_id))
    msg.percentage = level / 100.0 if level is not None else math.nan
    msg.voltage = volts if volts is not None else math.nan
    msg.temperature = temp if temp is not None else math.nan
    msg.current = ma / 1000.0 if ma is not None else math.nan
    msg.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING if charging == 1.0
                               else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if charging == 0.0
                               else BatteryState.POWER_SUPPLY_STATUS_UNKNOWN)
    msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
    msg.present = True
    msg.charge = msg.capacity = msg.design_capacity = math.nan
    return msg


def compressed_image(jpeg: bytes, seconds: float, frame_id: str) -> CompressedImage:
    """One camera frame as ``sensor_msgs/CompressedImage`` (JPEG), the form image_transport,
    RViz and Foxglove read on ``…/image/compressed``."""
    return CompressedImage(header=header(seconds, frame_id), format="jpeg", data=jpeg)


def compressed_video(unit: bytes, seconds: float, frame_id: str):
    """One H.264 access unit (Annex B; keyframes carry SPS/PPS) as ``foxglove_msgs/CompressedVideo``.
    Foxglove can only start decoding at a keyframe, so publish from the first keyframe on."""
    return CompressedVideo(timestamp=stamp(seconds), frame_id=frame_id, data=unit, format="h264")


def pcm16le(samples) -> bytes:
    """Signed 16-bit samples (``array.array("h")``) as little-endian bytes, the order ROS audio uses."""
    if sys.byteorder == "big":
        samples = type(samples)(samples.typecode, samples)
        samples.byteswap()
    return samples.tobytes()


def audio_info(rate: int):
    """``audio_common_msgs/AudioInfo`` for the phone's audio: mono, signed 16-bit little-endian PCM,
    in the form audio_capture publishes ("wave" = raw PCM without a header)."""
    return AudioInfo(channels=1, sample_rate=rate, sample_format="S16LE", bitrate=rate * 16, coding_format="wave")


def audio_data(pcm: bytes, seconds: float, frame_id: str):
    """One chunk of audio as ``(AudioData, AudioDataStamped)``; the stamp is the chunk's first sample."""
    data = AudioData(data=pcm)
    return data, AudioDataStamped(header=header(seconds, frame_id), audio=data)


def audio_array(pcm: bytes, rate: int) -> UInt8MultiArray:
    """Fallback without audio_common_msgs: the same bytes as ``std_msgs/UInt8MultiArray``, the format
    in the label (``pcm_s16le,16000Hz,mono``)."""
    msg = UInt8MultiArray()
    msg.layout.dim = [MultiArrayDimension(label=f"pcm_s16le,{rate}Hz,mono", size=len(pcm), stride=len(pcm))]
    msg.data = pcm
    return msg


class CombinedImu:
    """Merges accelerometer, gyroscope and an orientation sensor into one ``sensor_msgs/Imu``, the
    form robot_localization and most IMU consumers expect. Emits one message per accelerometer
    sample, with the latest gyroscope and orientation readings.

    Uses exactly one orientation source (mixing them would make the heading jump):
    GAME_ROTATION_VECTOR (default; no compass, so motors and steel don't disturb it, but yaw
    drifts slowly) or ROTATION_VECTOR (compass-based, absolute heading from north)."""

    def __init__(self, frame_id: str, orientation_type: int = GAME_ROTATION_VECTOR):
        self.frame_id = frame_id
        self.orientation_type = orientation_type
        self.angular_velocity: Optional[Vector3] = None
        self.orientation: Optional[Quaternion] = None

    def update(self, sensor_type: Optional[int], values: Sequence[Optional[float]], seconds: float) -> Optional[Imu]:
        if sensor_type == GYROSCOPE:
            self.angular_velocity = vector3(values)
        elif sensor_type == self.orientation_type:
            self.orientation = quaternion(values)
        elif sensor_type == ACCELEROMETER:
            msg = empty_imu(seconds, self.frame_id)
            msg.linear_acceleration = vector3(values)
            msg.linear_acceleration_covariance = [0.0] * 9
            if self.angular_velocity is not None:
                msg.angular_velocity = self.angular_velocity
                msg.angular_velocity_covariance = [0.0] * 9
            if self.orientation is not None:
                msg.orientation = self.orientation
                msg.orientation_covariance = [0.0] * 9
            return msg
        return None
