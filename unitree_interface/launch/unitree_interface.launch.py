from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'network_interface',
            default_value='enp86s0',
            description='Network interface for Unitree SDK communication'
        ),
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
            default_value='100',
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
        Node(
            package='unitree_interface',
            executable='unitree_interface_node',
            name='unitree_interface',
            output='screen',
            parameters=[{
                'network_interface': LaunchConfiguration('network_interface'),
                'motion_switcher_client_timeout': LaunchConfiguration('motion_switcher_client_timeout'),
                'loco_client_timeout': LaunchConfiguration('loco_client_timeout'),
                'audio_client_timeout': LaunchConfiguration('audio_client_timeout'),
                'volume': LaunchConfiguration('volume'),
                'ready_locomotion_stand_up_delay': LaunchConfiguration('ready_locomotion_stand_up_delay'),
                'ready_locomotion_start_delay': LaunchConfiguration('ready_locomotion_start_delay'),
            }],
        ),
    ])
