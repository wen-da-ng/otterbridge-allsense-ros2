# Changelog

## 0.1.0 (unreleased)

- First release: the AllSense phone as standard ROS 2 topics (Imu, MagneticField, FluidPressure, NavSatFix, BatteryState, …), a combined IMU, the robot frame from the app's mount, cameras (CompressedImage, or H.264 as foxglove CompressedVideo) and microphones (audio_common_msgs). Humble and Jazzy.
- The pairing token comes from `$ALLSENSE_TOKEN` or a `token_file` instead of a ROS parameter (readable by anyone on the DDS domain); a token left in `url` or `token` still works, is removed from the parameter, and logs a warning.
