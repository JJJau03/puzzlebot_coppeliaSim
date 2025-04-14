from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='coppelia_sim_ros2',
            executable='puzzlebot_controller',
            name='coppelia_ros_bridge',
            output='screen'
        ),
        
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rqt_image_view',
                    executable='rqt_image_view',
                    name='rqt_image_view',
                    arguments=['/camera/image_raw']
                )
            ]
        )
    ])