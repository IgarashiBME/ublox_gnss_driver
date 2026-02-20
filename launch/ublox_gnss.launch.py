"""Launch file for ublox_gnss_driver: starts rtk_rover_node and moving_base_node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'ntrip_port',
            default_value='/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00',
            description='Serial port for NTRIP/PVT receiver',
        ),
        DeclareLaunchArgument(
            'ntrip_baudrate',
            default_value='115200',
            description='Baud rate for NTRIP/PVT receiver',
        ),
        DeclareLaunchArgument(
            'moving_base_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for Moving Base Rover',
        ),
        DeclareLaunchArgument(
            'moving_base_baudrate',
            default_value='38400',
            description='Baud rate for Moving Base Rover',
        ),
        DeclareLaunchArgument(
            'ntrip_frame_id',
            default_value='gnss_link',
            description='frame_id for rtk_rover_node messages',
        ),
        DeclareLaunchArgument(
            'moving_base_frame_id',
            default_value='moving_base',
            description='frame_id for moving_base_node messages',
        ),

        # rtk_rover_node
        Node(
            package='ublox_gnss_driver',
            executable='rtk_rover_node',
            name='rtk_rover_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('ntrip_port'),
                'baudrate': LaunchConfiguration('ntrip_baudrate'),
                'frame_id': LaunchConfiguration('ntrip_frame_id'),
            }],
        ),

        # moving_base_node
        Node(
            package='ublox_gnss_driver',
            executable='moving_base_node',
            name='moving_base_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('moving_base_port'),
                'baudrate': LaunchConfiguration('moving_base_baudrate'),
                'frame_id': LaunchConfiguration('moving_base_frame_id'),
            }],
        ),
    ])
