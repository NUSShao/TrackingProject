import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

     # Include the robot_state_publisher launch file
    package_name='tracking_project'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
        )]),
        launch_arguments={'verbose': 'true'}.items(),
    )

    # Run the spawner node from the gazebo_ros package.
    spawn_entity_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic','/robot/robot_description',
                                   '-entity', 'my_bot',
                                   ],
                                   output='screen',
                                   #    !!! namespace added !!!
                                   namespace='robot',)
    
    # Run the spawner node from the gazebo_ros package.
    spawn_entity_target = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic','/target/robot_description',
                                   '-entity', 'target',
                                   '-x', '2.0'
                                   ],
                                   output='screen',
                                   #    !!! namespace added !!!
                                   namespace='target',)

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller"],
        # !!! namespace added !!!
        namespace='robot',
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"],
        # !!! namespace added !!!
        namespace='robot',
    )

    target_diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["target_diff_controller"],
        # !!! namespace added !!!
        namespace='target',
    )

    target_joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["target_joint_broadcaster"],
        # !!! namespace added !!!
        namespace='target',
    )
    
    # Launch everything!
    return LaunchDescription([
        # SetEnvironmentVariable('GAZEBO_MODEL_PATH', '/home/nusshao/.gazebo/models'),
        rsp,
        gazebo,
        # spawn_entity_robot,
        spawn_entity_target,
        # diff_drive_spawner,
        # joint_broad_spawner,
        target_diff_drive_spawner,
        target_joint_broad_spawner,
    ])

