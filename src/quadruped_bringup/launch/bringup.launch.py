from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate',   default_value='115200'),

        Node(package='quadruped_control', executable='keyboard_node',
             output='screen', emulate_tty=True),

        Node(package='quadruped_control', executable='gait_generator_node',
             output='screen',
             parameters=[{'imu_calib_samples': 50, 'z_ref': 150.0}]),

        Node(package='quadruped_control', executable='motor_control_node',
             output='screen',
             parameters=[{
                 'serial_port': LaunchConfiguration('serial_port'),
                 'baud_rate':   LaunchConfiguration('baud_rate'),
             }]),

        # record all comm + gait topics for drop analysis
        ExecuteProcess(cmd=[
            'ros2', 'bag', 'record',
            '/joint_angles', '/gait_cmd',
            '/teensy_ack', '/teensy_hb', '/comm_stats',
            '/imu',
            '-o', 'quadruped_bag'
        ], output='screen'),
    ])