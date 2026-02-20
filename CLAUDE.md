# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test Commands

```bash
# Source ROS2 and venv (pyproj requires venv)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/.venv/bin/activate

# Build this package (must build bme_common_msgs first if not already built)
cd ~/ros2_ws
colcon build --packages-select bme_common_msgs
source install/setup.bash
colcon build --packages-select ublox_gnss_driver

# Run lint tests
cd ~/ros2_ws
colcon test --packages-select ublox_gnss_driver
colcon test-result --verbose

# Launch both nodes
ros2 launch ublox_gnss_driver ublox_gnss.launch.py

# Run a single node
ros2 run ublox_gnss_driver rtk_rover_node --ros-args -p port:=/dev/ttyACM0
ros2 run ublox_gnss_driver moving_base_node --ros-args -p port:=/dev/ttyUSB0
```

## Architecture

This is an `ament_python` ROS2 Jazzy package that reads UBX binary protocol from u-blox GNSS receivers over serial and publishes parsed data as ROS2 topics.

### Module Roles

- **ubx_parser.py** — Stateless UBX protocol parser. Reads frames from a `serial.Serial` stream with sync-byte detection, Fletcher-8 checksum verification, and `struct.unpack_from` parsing into dataclasses (`NavPvt`, `NavHpposllh`, `NavRelposned`). Supports RELPOSNED v0 (40B) and v1 (64B) payloads automatically.

- **rtk_rover_node.py** — Node that reads NAV-PVT + NAV-HPPOSLLH from the primary GNSS receiver (RTK rover). Publishes 7 topics (PVT, HPPOSLLH, GnssSolution with UTM, NavSatFix, Odometry, GNGGA, gpstime). Subscribes to `ntrip_rtcm` (UInt8MultiArray) and writes RTCM bytes to the serial port for RTK corrections.

- **moving_base_node.py** — Node that reads NAV-RELPOSNED from a Moving Base Rover. Computes heading from the relative position NE vector as ENU yaw (`atan2(N, E)`). Publishes RELPOSNED, Imu (heading quaternion), and Odometry (UTM position + heading). Subscribes to `gnss_solution` from rtk_rover_node for UTM coordinates.

### Threading Model

Both nodes use the same pattern: `rclpy.spin()` on the main thread handles ROS2 callbacks (RTCM subscriber, GnssSolution subscriber), while a daemon thread runs `_serial_loop()` for blocking serial reads. Serial reconnection with 2-second retry on `SerialException`.

### Message Dependencies

Custom messages come from **bme_common_msgs** (sibling CMake package, must be built first):
- `PVT.msg` — Raw UBX-NAV-PVT fields + computed `fix_status`, `lon_deg`, `lat_deg`
- `HPPOSLLH.msg` — Raw UBX-NAV-HPPOSLLH fields (units in msg comments: lon/lat in deg*1e-7, accuracy in mm*0.1)
- `RELPOSNED.msg` — Raw UBX-NAV-RELPOSNED fields + computed `heading_rad`, `heading_deg`, `qgc_heading`
- `GnssSolution.msg` — Integrated solution with HP lat/lon in degrees, UTM coordinates, heading

### Key Design Decisions

- **UTM conversion** uses `pyproj.Proj` with lazy-init and zone caching (pyproj only available inside `~/ros2_ws/.venv`)
- **Quaternion from yaw** is computed inline (`sin(yaw/2), cos(yaw/2)`) to avoid a tf_transformations dependency
- **RTCM subscriber** uses `UInt8MultiArray` (raw bytes) instead of the ROS1 reference's hex-encoded String
- **fix_status** is extracted from `carrSoln` bits via bitwise shift: PVT uses bits 7..6, RELPOSNED uses bits 4..3
- **GnssSolution** combines PVT state (fix_status, num_sv) cached from the most recent PVT message with HPPOSLLH position data; heading fields are left at 0 (populated by moving_base_node's separate topic)

### Node Parameters

| Parameter | rtk_rover_node default | moving_base_node default |
|-----------|--------------------------|--------------------------|
| `port`    | `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00` | `/dev/ttyUSB0` |
| `baudrate`| 115200 | 38400 |
| `frame_id`| `gnss_link` | `moving_base` |
