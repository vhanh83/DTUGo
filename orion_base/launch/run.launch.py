from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    orion_base_dir = get_package_share_directory('orion_base')
    params_file = os.path.join(orion_base_dir, 'config', 'params.yaml')

    # Chạy launch URDF trước
    orion_urdf_dir = get_package_share_directory('orion_urdf')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                orion_urdf_dir,
                'launch',
                'view_robot.launch.py'
            )
        )
    )
    # Joystick Node - Node to read the raw PS5 Bluetooth device
    # Publishes to /joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[params_file],
    )

    joystick_parser_node = Node(
        package="orion_base",
        executable="joystick_parser_node",
        name="joystick_parser_node",
        output="screen",
        parameters=[params_file],
        remappings=[
            ('joy', '/joy'),
            ('joy_motion_cmd', '/joy_motion_cmd'),
            ('joy_eyes_cmd', '/joy_eyes_cmd'),
        ]
    )

    stm32_bridge_node = Node(
        package='orion_base',
        executable='stm32_bridge_node',
        name='stm32_bridge_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('orion_motion_cmd', '/orion_motion_cmd'),
            ('battery_voltage_front', '/battery_voltage_front'),
            ('battery_voltage_rear', '/battery_voltage_rear'),
            ('battery_voltage_jetson', '/battery_voltage_jetson'),
            ('imu_degrees', '/imu_degrees'),
            ('joint_angles_front_left', '/joint_angles_front_left'),
            ('joint_angles_front_right', '/joint_angles_front_right'),
            ('joint_angles_back_left', '/joint_angles_back_left'),
            ('joint_angles_back_right', '/joint_angles_back_right')
        ]
    )
    stm32_tcp_bridge_node = Node(
        package='orion_base',
        executable='stm32_tcp_bridge',
        name='stm32_tcp_bridge_node',
        output='screen',
        parameters=[
            params_file,
            {
                'esp32_ip': '192.168.5.227',
                'esp32_port': 5000,
            }
        ],
        remappings=[
            ('orion_motion_cmd', '/orion_motion_cmd'),
            ('battery_voltage_front', '/battery_voltage_front'),
            ('battery_voltage_rear', '/battery_voltage_rear'),
            ('battery_voltage_jetson', '/battery_voltage_jetson'),
            ('imu_degrees', '/imu_degrees'),
            ('joint_angles_front_left', '/joint_angles_front_left'),
            ('joint_angles_front_right', '/joint_angles_front_right'),
            ('joint_angles_back_left', '/joint_angles_back_left'),
            ('joint_angles_back_right', '/joint_angles_back_right'),
        ]
    )
    
    joint_state_republisher = Node(
        package='orion_base',
        executable='joint_state_republisher',
        name='joint_state_republisher',
        output='screen',
        parameters=[params_file],
    )

    roboeyes_node = Node(
        package='orion_base',
        executable='roboeyes_node',
        name='roboeyes_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('orion_eyes_cmd', '/orion_eyes_cmd'),
        ]
    )

    cmd_mux_node = Node(
        package='orion_base',
        executable='cmd_mux_node',
        name='cmd_mux_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('joy_motion_cmd', '/joy_motion_cmd'),
            ('joy_eyes_cmd', '/joy_eyes_cmd'),
            ('nav_motion_cmd', '/nav_motion_cmd'),
            ('orion_motion_cmd', '/orion_motion_cmd'),
            ('orion_eyes_cmd', '/orion_eyes_cmd'),
        ]
    )

    foxglove_bridge_node = Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765, 'address': '0.0.0.0'}],
        )
    # WebSocket Bridge Node
    webclient_bridge_node = Node(
        package='orion_base',
        executable='webclient_bridge',
        name='webclient_bridge',
        output='screen',
        parameters=[params_file],
    )
    orion_base_dir = get_package_share_directory('orion_base')

    rviz_config = os.path.join(
        orion_base_dir,
        'rviz',
        'default.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    return LaunchDescription([
        # Bước 1: Khởi động URDF
        robot_description_launch,

        # Bước 2: Sau 3 giây, khởi động các node còn lại
        TimerAction(
            period=3.0,
            actions=[
                stm32_tcp_bridge_node,
                joint_state_republisher,
                joy_node,
                joystick_parser_node,
                cmd_mux_node,
                webclient_bridge_node,
                rviz_node
            ]
        ),
    ])