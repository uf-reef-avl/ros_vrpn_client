from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_vrpn_client',
            executable='ros_vrpn_client',
            name='rail',
            parameters=[
                {'vrpn_ip': '192.168.1.104'},
                {'my_int': 3883}
            ],
            output='screen',
            emulate_tty=True
        )
    ])