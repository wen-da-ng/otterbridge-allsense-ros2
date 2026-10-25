import array
import math

import pytest

from sensor_msgs.msg import FluidPressure, Imu, MagneticField, Range
from std_msgs.msg import Float64MultiArray

from allsense_ros2 import convert


def test_stamp_splits_seconds():
    t = convert.stamp(1700000000.25)
    assert t.sec == 1700000000 and t.nanosec == 250_000_000


def test_accelerometer_is_imu_with_only_acceleration():
    m = convert.to_message(convert.ACCELEROMETER, (0.1, 0.2, 9.81), 10.5, "phone_sensors")
    assert isinstance(m, Imu) and m.header.frame_id == "phone_sensors"
    assert (m.linear_acceleration.x, m.linear_acceleration.z) == (0.1, 9.81)
    assert m.linear_acceleration_covariance[0] == 0.0
    assert m.orientation_covariance[0] == -1.0 and m.angular_velocity_covariance[0] == -1.0


def test_magnetometer_microtesla_to_tesla():
    m = convert.to_message(convert.MAGNETIC_FIELD, (20.0, -5.0, -40.0), 0, "f")
    assert isinstance(m, MagneticField)
    assert math.isclose(m.magnetic_field.x, 20e-6) and math.isclose(m.magnetic_field.z, -40e-6)


def test_pressure_hpa_to_pa():
    m = convert.to_message(convert.PRESSURE, (1013.25,), 0, "f")
    assert isinstance(m, FluidPressure) and math.isclose(m.fluid_pressure, 101325.0)


def test_rotation_vector_quaternion_and_heading_variance():
    s = math.sqrt(0.5)
    m = convert.to_message(convert.ROTATION_VECTOR, (0.0, 0.0, s, s, 0.1), 0, "f")
    assert math.isclose(m.orientation.z, s) and math.isclose(m.orientation.w, s)
    assert math.isclose(m.orientation_covariance[8], 0.01)
    assert m.linear_acceleration_covariance[0] == -1.0


def test_rotation_vector_without_w():
    q = convert.quaternion((0.0, 0.0, 0.6))
    assert math.isclose(q.w, 0.8)


def test_proximity_cm_to_m():
    m = convert.to_message(convert.PROXIMITY, (5.0,), 0, "f", max_range_m=0.05)
    assert isinstance(m, Range) and math.isclose(m.range, 0.05) and m.max_range == 0.05


def test_unknown_types_are_labelled_arrays():
    m = convert.to_message(65600, (1.0, None), 0, "f", fields=("a", "b"))
    assert isinstance(m, Float64MultiArray)
    assert m.layout.dim[0].label == "a,b" and math.isnan(m.data[1])


def test_combined_imu_uses_one_orientation_source():
    imu = convert.CombinedImu("f")  # game rotation vector
    assert imu.update(convert.GYROSCOPE, (0.1, 0.2, 0.3), 0) is None
    assert imu.update(convert.ROTATION_VECTOR, (0.0, 0.0, 1.0, 0.0), 0) is None  # ignored: not the chosen source
    assert imu.update(convert.GAME_ROTATION_VECTOR, (0.0, 0.0, 0.0, 1.0), 0) is None
    m = imu.update(convert.ACCELEROMETER, (0.0, 0.0, 9.81), 1.0)
    assert m.angular_velocity.z == 0.3 and m.orientation.w == 1.0 and m.linear_acceleration.z == 9.81
    assert m.orientation_covariance[0] == 0.0 and m.angular_velocity_covariance[0] == 0.0


def test_combined_imu_before_gyro_marks_fields_missing():
    m = convert.CombinedImu("f").update(convert.ACCELEROMETER, (0.0, 0.0, 9.81), 1.0)
    assert m.angular_velocity_covariance[0] == -1.0 and m.orientation_covariance[0] == -1.0


def test_gnss_position_is_navsatfix():
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    msg = convert.to_message(convert.GNSS_POSITION, [51.477928123456, -0.001545987654, 61.5, 4.0, 0.0, None, 6.0], 12.5, "phone")
    assert isinstance(msg, NavSatFix)
    assert msg.latitude == 51.477928123456 and msg.longitude == -0.001545987654 and msg.altitude == 61.5
    assert msg.status.status == NavSatStatus.STATUS_FIX
    assert msg.position_covariance[0] == 16.0 and msg.position_covariance[8] == 36.0
    assert msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    no_fix = convert.to_message(convert.GNSS_POSITION, [None, None, None, None, None, None, None], 1.0, "phone")
    assert no_fix.status.status == NavSatStatus.STATUS_NO_FIX


def test_battery_is_battery_state():
    from sensor_msgs.msg import BatteryState

    msg = convert.to_message(convert.BATTERY, [87.0, 4.012, 31.5, -412.0, 0.0], 1.0, "phone")
    assert isinstance(msg, BatteryState)
    assert abs(msg.percentage - 0.87) < 1e-6 and abs(msg.current + 0.412) < 1e-6 and msg.present
    assert msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING


def test_kind_uses_stream_type_for_non_sensors():
    class Info:
        sensor_type = None
        stream_type = "allsense.gnss.position"

    assert convert.kind(Info()) == convert.GNSS_POSITION
    Info.sensor_type = 1
    assert convert.kind(Info()) == 1


def test_camera_frame_is_compressed_image():
    from sensor_msgs.msg import CompressedImage

    msg = convert.compressed_image(b"\xff\xd8jpeg\xff\xd9", 12.25, "phone_camera_back")
    assert isinstance(msg, CompressedImage) and msg.format == "jpeg"
    assert bytes(msg.data) == b"\xff\xd8jpeg\xff\xd9" and msg.header.stamp.sec == 12 and msg.header.frame_id == "phone_camera_back"


def test_h264_unit_is_compressed_video():
    if convert.CompressedVideo is None:
        pytest.skip("foxglove_msgs isn't installed")
    unit = b"\x00\x00\x00\x01\x67sps\x00\x00\x00\x01\x68pps\x00\x00\x00\x01\x65idr"
    msg = convert.compressed_video(unit, 12.25, "phone_camera_back")
    assert type(msg).__name__ == "CompressedVideo" and msg.format == "h264" and bytes(msg.data) == unit
    assert msg.timestamp.sec == 12 and msg.timestamp.nanosec == 250_000_000 and msg.frame_id == "phone_camera_back"


def test_pcm16le_is_little_endian():
    assert convert.pcm16le(array.array("h", [1, -2, 0x1234])) == b"\x01\x00\xfe\xff\x34\x12"


def test_audio_chunk_is_audio_common():
    if convert.AudioData is None:
        pytest.skip("audio_common_msgs isn't installed")
    info = convert.audio_info(16000)
    assert (info.channels, info.sample_rate, info.sample_format, info.coding_format) == (1, 16000, "S16LE", "wave")
    data, stamped = convert.audio_data(b"\x01\x00\xfe\xff", 3.5, "phone_mic_mic")
    assert bytes(data.data) == b"\x01\x00\xfe\xff" and bytes(stamped.audio.data) == b"\x01\x00\xfe\xff"
    assert stamped.header.stamp.sec == 3 and stamped.header.stamp.nanosec == 500_000_000 and stamped.header.frame_id == "phone_mic_mic"


def test_audio_fallback_is_labelled_bytes():
    msg = convert.audio_array(b"\x01\x00\xfe\xff", 16000)
    assert bytes(msg.data) == b"\x01\x00\xfe\xff" and msg.layout.dim[0].label == "pcm_s16le,16000Hz,mono"
