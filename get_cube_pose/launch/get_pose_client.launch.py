from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_grasping',
            executable='basic_grasping_perception_node',
            name='basic_grasping_perception_node',
            output='screen',
            parameters=[{'debug_topics': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('get_cube_pose'), 'rviz_config', 'point_viz.rviz'])],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='get_cube_pose',
                    executable='get_pose_client',
                    name='get_pose_client',
                    output='screen'
                ),
        ])
    ])
