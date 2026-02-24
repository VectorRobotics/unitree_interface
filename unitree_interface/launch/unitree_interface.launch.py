from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'motion_switcher_client_timeout',
            default_value='5.0',
            description='Timeout for MotionSwitcherClient (seconds)'
        ),
        DeclareLaunchArgument(
            'loco_client_timeout',
            default_value='10.0',
            description='Timeout for LocoClient (seconds)'
        ),
        DeclareLaunchArgument(
            'audio_client_timeout',
            default_value='5.0',
            description='Timeout for AudioClient (seconds)'
        ),
        DeclareLaunchArgument(
            'volume',
            default_value='80',
            description='Volume for the speaker'
        ),
        DeclareLaunchArgument(
            'ready_locomotion_stand_up_delay',
            default_value='5',
            description='Delay before calling stand_up() after damp() in the ready locomotion sequence (seconds)'
        ),
        DeclareLaunchArgument(
            'ready_locomotion_start_delay',
            default_value='10',
            description='Delay before calling start() after stand_up() in the ready locomotion sequence (seconds)'
        ),
        DeclareLaunchArgument(
            'release_arms_steps',
            default_value='250',
            description='Number of steps over which to return control of arms to the Unitree locomotion policy'
        ),
        DeclareLaunchArgument(
            'release_arms_interval_ms',
            default_value='20',
            description='Delay between subsequent messages to release control of arms (milliseconds)'
        ),
        Node(
            package='unitree_interface',
            executable='unitree_interface_node',
            name='unitree_interface',
            output='screen',
            parameters=[{
                'motion_switcher_client_timeout': LaunchConfiguration('motion_switcher_client_timeout'),
                'loco_client_timeout': LaunchConfiguration('loco_client_timeout'),
                'audio_client_timeout': LaunchConfiguration('audio_client_timeout'),
                'volume': LaunchConfiguration('volume'),
                'ready_locomotion_stand_up_delay': LaunchConfiguration('ready_locomotion_stand_up_delay'),
                'ready_locomotion_start_delay': LaunchConfiguration('ready_locomotion_start_delay'),
                'release_arms_steps': LaunchConfiguration('release_arms_steps'),
                'release_arms_interval_ms': LaunchConfiguration('release_arms_interval_ms'),
            }],
            remappings=[
                ('/unitree_interface/cmd_arm', '/joint_states'),
                ('/unitree_interface/cmd_low', '/joint_states'),
                ('/unitree_interface/joint_states', '/feedback'),
            ],
        ),
    ])
