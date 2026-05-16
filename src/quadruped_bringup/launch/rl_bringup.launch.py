from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),

        Node(package='quadruped_control', executable='keyboard_node',
             output='screen', emulate_tty=True),

        Node(package='quadruped_control', executable='policy_node',
             output='screen',
             parameters=[{
                 'output_topic': '/joint_angles_unsafe',
             }]),

        Node(package='quadruped_control', executable='safety_node',
             output='screen',
             parameters=[{
                 'input_topic': '/joint_angles_unsafe',
                 'output_topic': '/joint_angles',
             }]),

        Node(package='quadruped_control', executable='joint_state_bridge_node',
             output='screen',
             parameters=[{
                 'input_topic': '/joint_angles',
             }]),

        Node(package='quadruped_control', executable='motor_control_node',
             output='screen',
             parameters=[{
                 'serial_port': LaunchConfiguration('serial_port'),
                 'baud_rate': LaunchConfiguration('baud_rate'),
             }]),

        ExecuteProcess(cmd=[
            'ros2', 'bag', 'record',
            '/joint_angles_unsafe', '/joint_angles', '/joint_states',
            '/gait_cmd',
            '/teensy_ack', '/teensy_hb', '/comm_stats',
            '/imu',
            '-o', 'quadruped_rl_bag'
        ], output='screen'),
    ])
