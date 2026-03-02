"""ROS2 node for u-blox Moving Base Rover (RELPOSNED).

Reads UBX-NAV-RELPOSNED from a serial port, computes heading from
the relative position vector, and publishes Imu and Odometry messages
combined with position data from the rtk_rover_node.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
import serial

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from bme_common_msgs.msg import GnssSolution, HPPOSLLH, PVT, RELPOSNED

from ublox_gnss_driver.ubx_parser import (
    UBX_SYNC_1,
    UBX_NAV_CLASS,
    UBX_NAV_RELPOSNED_ID,
    NavRelposned,
    read_ubx_frame,
    parse_nav_relposned,
)

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 38400
DEFAULT_FRAME_ID = 'moving_base'


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Convert yaw angle (rad) to quaternion (x, y, z, w).

    Assumes roll=0, pitch=0.
    """
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MovingBaseNode(Node):
    """ROS2 node that reads UBX NAV-RELPOSNED from a u-blox Moving Base Rover."""

    def __init__(self) -> None:
        super().__init__('moving_base_node')

        # Parameters
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)

        self._port: str = self.get_parameter('port').value
        self._baudrate: int = int(self.get_parameter('baudrate').value)
        self._frame_id: str = self.get_parameter('frame_id').value

        # Publishers
        self._pub_relposned = self.create_publisher(RELPOSNED, 'relposned', 10)
        self._pub_heading_imu = self.create_publisher(Imu, 'heading_imu', 10)
        self._pub_gnss_odom = self.create_publisher(Odometry, 'gnss_odom', 10)
        self._pub_gnss_solution = self.create_publisher(GnssSolution, 'gnss_solution', 10)

        # Subscribe to PVT and HPPOSLLH from rtk_rover_node
        self.create_subscription(PVT, 'pvt', self._pvt_callback, 10)
        self.create_subscription(
            HPPOSLLH, 'hpposllh', self._hpposllh_callback, 10)

        # Cached state from PVT
        self._cached_fix_status: int = 0
        self._cached_num_sv: int = 0

        # Cached position (built from HPPOSLLH, heading fields left at 0)
        self._cached_position: GnssSolution | None = None

        # UTM converter (lazy-init)
        self._utm_proj = None
        self._utm_zone: int = 0

        # Serial port
        self._ser: serial.Serial | None = None

        # Start serial reading thread
        self._thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'moving_base_node started: port={self._port}, '
            f'baudrate={self._baudrate}, frame_id={self._frame_id}')

    # ---- Serial connection management ----

    def _connect_serial(self) -> None:
        """Open (or reopen) the serial port."""
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1.0,
            )
            self.get_logger().info(f'Serial port opened: {self._port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self._ser = None

    def _serial_loop(self) -> None:
        """Main serial reading loop (runs in a dedicated thread)."""
        while rclpy.ok():
            try:
                if self._ser is None or not self._ser.is_open:
                    self._connect_serial()
                    if self._ser is None:
                        time.sleep(2.0)
                        continue

                byte = self._ser.read(1)
                if not byte:
                    continue  # timeout

                if byte[0] == UBX_SYNC_1:
                    frame = read_ubx_frame(self._ser)
                    if (frame and
                            frame.msg_class == UBX_NAV_CLASS and
                            frame.msg_id == UBX_NAV_RELPOSNED_ID):
                        relposned = parse_nav_relposned(frame.payload)
                        if relposned:
                            self._handle_relposned(relposned)

            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}. Reconnecting...')
                if self._ser is not None:
                    try:
                        self._ser.close()
                    except Exception:
                        pass
                self._ser = None
                time.sleep(2.0)
            except Exception as e:
                self.get_logger().error(f'Unexpected error in serial loop: {e}')
                time.sleep(0.1)

    # ---- RELPOSNED handler ----

    def _handle_relposned(self, rp: NavRelposned) -> None:
        """Process parsed RELPOSNED and publish all output topics."""
        now = self.get_clock().now().to_msg()

        # Publish RELPOSNED message
        msg = RELPOSNED()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.version = rp.version
        msg.ref_station_id = rp.ref_station_id
        msg.itow = rp.itow
        msg.rel_pos_n = rp.rel_pos_n
        msg.rel_pos_e = rp.rel_pos_e
        msg.rel_pos_d = rp.rel_pos_d
        msg.rel_pos_length = rp.rel_pos_length
        msg.rel_pos_heading = rp.rel_pos_heading
        msg.rel_pos_hpn = rp.rel_pos_hpn
        msg.rel_pos_hpe = rp.rel_pos_hpe
        msg.rel_pos_hpd = rp.rel_pos_hpd
        msg.rel_pos_hp_length = rp.rel_pos_hp_length
        msg.acc_n = rp.acc_n
        msg.acc_e = rp.acc_e
        msg.acc_d = rp.acc_d
        msg.acc_length = rp.acc_length
        msg.acc_heading = rp.acc_heading
        msg.flags = rp.flags
        msg.fix_status = rp.fix_status
        msg.heading_rad = rp.heading_rad
        msg.heading_deg = rp.heading_deg
        msg.qgc_heading = rp.qgc_heading
        self._pub_relposned.publish(msg)

        # Convert heading to quaternion
        qx, qy, qz, qw = quaternion_from_yaw(rp.heading_rad)

        # Publish Imu (heading as orientation quaternion)
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self._frame_id
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        # Set covariance to -1 to indicate gyro/accel are not available
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self._pub_heading_imu.publish(imu_msg)

        # Publish integrated GnssSolution (position + heading)
        cached = self._cached_position
        if cached is not None:
            gnss_msg = GnssSolution()
            gnss_msg.header.stamp = now
            gnss_msg.header.frame_id = cached.header.frame_id
            gnss_msg.itow = cached.itow
            gnss_msg.num_sv = cached.num_sv
            gnss_msg.position_rtk_status = cached.position_rtk_status
            gnss_msg.longitude = cached.longitude
            gnss_msg.latitude = cached.latitude
            gnss_msg.utm_easting = cached.utm_easting
            gnss_msg.utm_northing = cached.utm_northing
            gnss_msg.height = cached.height
            gnss_msg.h_acc = cached.h_acc
            gnss_msg.v_acc = cached.v_acc
            gnss_msg.heading_deg = rp.heading_deg
            gnss_msg.heading_rtk_status = rp.fix_status
            self._pub_gnss_solution.publish(gnss_msg)

        # Publish Odometry (UTM position + heading)
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self._frame_id
        if cached is not None:
            odom_msg.pose.pose.position.x = cached.utm_easting
            odom_msg.pose.pose.position.y = cached.utm_northing
            odom_msg.pose.pose.position.z = cached.height
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        self._pub_gnss_odom.publish(odom_msg)

    # ---- PVT / HPPOSLLH callbacks ----

    def _pvt_callback(self, msg: PVT) -> None:
        """Cache fix status and satellite count from rtk_rover_node's PVT."""
        self._cached_fix_status = msg.fix_status
        self._cached_num_sv = msg.num_sv

    def _hpposllh_callback(self, msg: HPPOSLLH) -> None:
        """Build and cache a position-only GnssSolution from HPPOSLLH."""
        lon_hp_deg = msg.lon * 1e-7 + msg.lon_hp * 1e-9
        lat_hp_deg = msg.lat * 1e-7 + msg.lat_hp * 1e-9
        height_hp_m = msg.height * 1e-3 + msg.height_hp * 1e-4
        easting, northing = self._to_utm(lat_hp_deg, lon_hp_deg)

        gnss = GnssSolution()
        gnss.header.stamp = msg.header.stamp
        gnss.header.frame_id = msg.header.frame_id
        gnss.itow = msg.itow
        gnss.num_sv = self._cached_num_sv
        gnss.position_rtk_status = self._cached_fix_status
        gnss.longitude = lon_hp_deg
        gnss.latitude = lat_hp_deg
        gnss.utm_easting = easting
        gnss.utm_northing = northing
        gnss.height = height_hp_m
        gnss.h_acc = round(msg.h_acc * 0.1)  # 0.1mm -> mm (uint32)
        gnss.v_acc = round(msg.v_acc * 0.1)
        self._cached_position = gnss

    # ---- UTM conversion ----

    def _to_utm(self, lat: float, lon: float) -> tuple[float, float]:
        """Convert latitude/longitude (degrees) to UTM easting/northing."""
        try:
            from pyproj import Proj
        except ImportError:
            self.get_logger().error(
                'pyproj is not available. Activate the venv: '
                'source ros2_ws/.venv/bin/activate')
            return 0.0, 0.0

        zone = int((lon + 180.0) / 6.0) + 1
        if zone != self._utm_zone or self._utm_proj is None:
            self._utm_zone = zone
            self._utm_proj = Proj(proj='utm', zone=zone, ellps='WGS84')

        easting, northing = self._utm_proj(lon, lat)
        return easting, northing

    # ---- Cleanup ----

    def destroy_node(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MovingBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
