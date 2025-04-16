from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the YAML launch file
    config = os.path.join(get_package_share_directory('vrpn_mocap'), 'config', 'client.yaml')
    print('Config file found')
    # Include the existing launch file from Package A (vrpn_mocap)
    vrpn_node = Node(
        package='vrpn_mocap',
        executable='client_node',
        namespace='vrpn_mocap',
        name='vrpn_mocap_client_node',
        parameters=[config, {'server': '192.168.0.2', 'port': '3883'}]
    )    


    # Start the node from Package B after 2 seconds
    delayed_opti = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'opti_track', 'opti_data_sub'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        vrpn_node,
        delayed_opti
    ])
