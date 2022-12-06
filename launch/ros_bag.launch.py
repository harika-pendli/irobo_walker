from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    
    record_all_topics = LaunchConfiguration('record_all_topics')

    return LaunchDescription([

        DeclareLaunchArgument(
            'record_all_topics',
            default_value='False'
        ),

        Node(
            package='irobo_walker',
            executable='navigate',
            name='irobo_walker',
            parameters=[{
                "record_all_topics": LaunchConfiguration('record_all_topics'),
            }]
        ),

        ExecuteProcess(
        condition=IfCondition(record_all_topics),
        cmd=[
            'ros2', 'bag', 'record', '-o all_topics_bag', '-a', '-x', 'demo_cam/camera/.*'
        ],
        shell=True
        )

    ])