from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # crazyflie/launch/launch.py 경로 설정
    crazyflie_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('crazyflie'), 'launch', 'launch.py')
        )
    )

    # 10초 후 실행될 cf_bag 노드
    delayed_cf_bag = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'cf_bag', 'cf_bag'],
                output='screen'
            )
        ]
    )

    # 10초 후 실행될 cmd_position 노드
    delayed_cmd_position = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'cmd', 'cmd_position'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        crazyflie_launch,
        delayed_cf_bag,
        delayed_cmd_position
    ])
