"""ROS2 node for u-blox RTK rover receiver.

Reads UBX-NAV-PVT and UBX-NAV-HPPOSLLH from a serial port,
publishes parsed data as ROS2 topics, and writes RTCM correction
data received via subscription back to the serial port.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
import serial

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String, UInt8MultiArray

from bme_common_msgs.msg import GnssSolution, HPPOSLLH, PVT

from ublox_gnss_driver.ubx_parser import (
    UBX_SYNC_1,
    UBX_NAV_CLASS,
    UBX_NAV_PVT_ID,
    UBX_NAV_HPPOSLLH_ID,
    NavPvt,
    NavHpposllh,
    read_ubx_frame,
    parse_nav_pvt,
    parse_nav_hpposllh,
)

# Default serial port for the RTK rover receiver
DEFAULT_PORT = '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'
DEFAULT_BAUDRATE = 115200
DEFAULT_FRAME_ID = 'gnss_link'


class RtkRoverNode(Node):
    """ROS2 node that reads UBX NAV-PVT and NAV-HPPOSLLH from a u-blox receiver."""

    def __init__(self) -> None:
        super().__init__('rtk_rover_node')

        # Parameters
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('frame_id', DEFAULT_FRAME_ID)

        self._port: str = self.get_parameter('port').value
        self._baudrate: int = int(self.get_parameter('baudrate').value)
        self._frame_id: str = self.get_parameter('frame_id').value

        # Publishers
        self._pub_pvt = self.create_publisher(PVT, 'pvt', 10)
        self._pub_hpposllh = self.create_publisher(HPPOSLLH, 'hpposllh', 10)
        self._pub_gnss_solution = self.create_publisher(GnssSolution, 'gnss_solution', 10)
        self._pub_navsatfix = self.create_publisher(NavSatFix, 'navsatfix', 10)
        self._pub_utm = self.create_publisher(Odometry, 'utm', 10)
        self._pub_gngga = self.create_publisher(String, 'gngga', 10)
        self._pub_gpstime = self.create_publisher(String, 'gpstime', 10)

        # Subscriber for RTCM correction data
        self.create_subscription(
            UInt8MultiArray, 'ntrip_rtcm', self._rtcm_callback, 10)

        # State shared between PVT and HPPOSLLH handlers
        self._fix_status: int = 0
        self._num_sv: int = 0

        # UTM converter (lazy-init)
        self._utm_proj = None
        self._utm_zone: int = 0

        # Serial port
        self._ser: serial.Serial | None = None
        self._serial_lock = threading.Lock()

        # Start serial reading thread
        self._thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'rtk_rover_node started: port={self._port}, '
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
                    if frame and frame.msg_class == UBX_NAV_CLASS:
                        self._process_nav_message(frame.msg_id, frame.payload)
                elif byte == b'$':
                    self._process_nmea()

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

    # ---- UBX message processing ----

    def _process_nav_message(self, msg_id: int, payload: bytes) -> None:
        """Dispatch NAV-class messages to the appropriate handler."""
        if msg_id == UBX_NAV_PVT_ID:
            pvt = parse_nav_pvt(payload)
            if pvt:
                self._handle_pvt(pvt)
        elif msg_id == UBX_NAV_HPPOSLLH_ID:
            hpposllh = parse_nav_hpposllh(payload)
            if hpposllh:
                self._handle_hpposllh(hpposllh)

    def _handle_pvt(self, pvt: NavPvt) -> None:
        """Process NAV-PVT data and publish PVT message."""
        now = self.get_clock().now().to_msg()

        # Update shared state for HPPOSLLH handler
        self._fix_status = pvt.fix_status
        self._num_sv = pvt.num_sv

        # Publish PVT message
        msg = PVT()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.itow = pvt.itow
        msg.year = pvt.year
        msg.month = pvt.month
        msg.day = pvt.day
        msg.hour = pvt.hour
        msg.min = pvt.minute
        msg.sec = pvt.sec
        msg.valid = pvt.valid
        msg.t_acc = pvt.t_acc
        msg.nano = pvt.nano
        msg.fix_type = pvt.fix_type
        msg.flags = pvt.flags
        msg.flags2 = pvt.flags2
        msg.num_sv = pvt.num_sv
        msg.lon = pvt.lon
        msg.lat = pvt.lat
        msg.height = pvt.height
        msg.h_msl = pvt.h_msl
        msg.h_acc = pvt.h_acc
        msg.v_acc = pvt.v_acc
        msg.vel_n = pvt.vel_n
        msg.vel_e = pvt.vel_e
        msg.vel_d = pvt.vel_d
        msg.g_speed = pvt.g_speed
        msg.head_mot = pvt.head_mot
        msg.s_acc = pvt.s_acc
        msg.head_acc = pvt.head_acc
        msg.p_dop = pvt.p_dop
        msg.flags3 = pvt.flags3
        msg.head_veh = pvt.head_veh
        msg.mag_dec = pvt.mag_dec
        msg.mag_acc = pvt.mag_acc
        msg.fix_status = pvt.fix_status
        msg.lon_deg = pvt.lon_deg
        msg.lat_deg = pvt.lat_deg
        self._pub_pvt.publish(msg)

        # Publish GPS time
        time_str = f'{self.get_clock().now().nanoseconds},{pvt.itow}'
        gpstime_msg = String()
        gpstime_msg.data = time_str
        self._pub_gpstime.publish(gpstime_msg)

    def _handle_hpposllh(self, hp: NavHpposllh) -> None:
        """Process NAV-HPPOSLLH data and publish HPPOSLLH, GnssSolution, NavSatFix, Odometry."""
        now = self.get_clock().now().to_msg()

        # Compute high-precision coordinates
        lon_hp_deg = hp.lon * 1e-7 + hp.lon_hp * 1e-9
        lat_hp_deg = hp.lat * 1e-7 + hp.lat_hp * 1e-9
        height_hp_m = hp.height * 1e-3 + hp.height_hp * 1e-4

        # Accuracy in meters
        h_acc_m = hp.h_acc * 1e-4   # raw is 0.1mm → m
        v_acc_m = hp.v_acc * 1e-4

        # Publish HPPOSLLH message (raw UBX fields)
        hpposllh_msg = HPPOSLLH()
        hpposllh_msg.header.stamp = now
        hpposllh_msg.header.frame_id = self._frame_id
        hpposllh_msg.version = hp.version
        hpposllh_msg.flags = hp.flags
        hpposllh_msg.itow = hp.itow
        hpposllh_msg.lon = hp.lon
        hpposllh_msg.lat = hp.lat
        hpposllh_msg.height = hp.height
        hpposllh_msg.h_msl = hp.h_msl
        hpposllh_msg.lon_hp = hp.lon_hp
        hpposllh_msg.lat_hp = hp.lat_hp
        hpposllh_msg.height_hp = hp.height_hp
        hpposllh_msg.h_msl_hp = hp.h_msl_hp
        hpposllh_msg.h_acc = hp.h_acc
        hpposllh_msg.v_acc = hp.v_acc
        self._pub_hpposllh.publish(hpposllh_msg)

        # UTM conversion
        easting, northing = self._to_utm(lat_hp_deg, lon_hp_deg)

        # Publish GnssSolution
        gnss_msg = GnssSolution()
        gnss_msg.header.stamp = now
        gnss_msg.header.frame_id = self._frame_id
        gnss_msg.itow = hp.itow
        gnss_msg.num_sv = self._num_sv
        gnss_msg.position_rtk_status = self._fix_status
        gnss_msg.longitude = lon_hp_deg
        gnss_msg.latitude = lat_hp_deg
        gnss_msg.utm_easting = easting
        gnss_msg.utm_northing = northing
        gnss_msg.height = height_hp_m
        gnss_msg.h_acc = round(hp.h_acc * 0.1)  # 0.1mm → mm (uint32)
        gnss_msg.v_acc = round(hp.v_acc * 0.1)
        gnss_msg.heading_rtk_status = 0
        gnss_msg.heading_deg = 0.0
        self._pub_gnss_solution.publish(gnss_msg)

        # Publish NavSatFix
        navsat_msg = NavSatFix()
        navsat_msg.header.stamp = now
        navsat_msg.header.frame_id = self._frame_id
        navsat_msg.status.status = self._fix_status
        navsat_msg.status.service = NavSatStatus.SERVICE_GPS
        navsat_msg.latitude = lat_hp_deg
        navsat_msg.longitude = lon_hp_deg
        navsat_msg.altitude = height_hp_m
        navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        navsat_msg.position_covariance = [
            (h_acc_m * h_acc_m) / 2.0, 0.0, 0.0,
            0.0, (h_acc_m * h_acc_m) / 2.0, 0.0,
            0.0, 0.0, (v_acc_m * v_acc_m),
        ]
        self._pub_navsatfix.publish(navsat_msg)

        # Publish Odometry (UTM coordinates)
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self._frame_id
        odom_msg.pose.pose.position.x = easting
        odom_msg.pose.pose.position.y = northing
        odom_msg.pose.pose.position.z = height_hp_m
        self._pub_utm.publish(odom_msg)

    # ---- NMEA processing ----

    def _process_nmea(self) -> None:
        """Read and publish GNGGA NMEA sentence."""
        if self._ser is None:
            return
        line = self._ser.readline()
        if not line:
            return
        try:
            nmea_str = ('$' + line.decode('ascii', errors='ignore')).strip()
        except Exception:
            return

        if len(nmea_str) > 6 and nmea_str[1:6] == 'GNGGA':
            msg = String()
            msg.data = nmea_str
            self._pub_gngga.publish(msg)

    # ---- RTCM callback ----

    def _rtcm_callback(self, msg: UInt8MultiArray) -> None:
        """Write received RTCM data to the serial port."""
        if self._ser is not None and self._ser.is_open:
            try:
                with self._serial_lock:
                    self._ser.write(bytes(msg.data))
            except serial.SerialException as e:
                self.get_logger().warn(f'Failed to write RTCM to serial: {e}')

    # ---- UTM conversion ----

    def _to_utm(self, lat: float, lon: float) -> tuple[float, float]:
        """Convert latitude/longitude (degrees) to UTM easting/northing (meters)."""
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
    node = RtkRoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
