from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Set the TURTLEBOT3_MODEL environment variable
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle_pi'),

        # Include the slam_toolbox launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            )
        ),
        
        # Include the external launch file from the turtlebot3_gazebo package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtle_tech.launch.py')
            )
        ),

        # Include the nav2_bringup RViz launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items()
        ),

        # Start the autonomous_exploration control node
        Node(
            package='autonomous_exploration',
            executable='control',
            name='autonomous_exploration_control',
            output='screen'
        ),

        # Start the rgbDetect node
        Node(
            package='circle_detection',
            executable='circle_detector',
            name='cylinder_detector',
            output='screen',
            parameters=[]  # Add if you have any parameter files to load
        )
    ])