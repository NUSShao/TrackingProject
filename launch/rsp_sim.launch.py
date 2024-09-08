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
                                   remappings=[("/robot_state_publisher", "/robot/robot_state_publisher")]
                                   )
    
    # Run the spawner node from the gazebo_ros package.
    spawn_entity_target = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic','/target/robot_description',
                                   '-entity', 'target',
                                   '-x', '2.0'
                                   ],
                                   output='screen',
                                   remappings=[("/robot_state_publisher", "/target/robot_state_publisher")],
                                   )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # !!! namespace added !!!
        namespace='robot',
        name="robot_spawn_controller_joint_state_broadcaster",
        arguments=["robot_diff_controller", "--controller-manager", "/robot/controller_manager"],
        output="screen",
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # !!! namespace added !!!
        namespace='robot',
        name="robot_spawn_controller_joint_state_broadcaster",
        arguments=["robot_joint_broadcaster", "--controller-manager", "/robot/controller_manager"],
        output="screen",
    )

    target_diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # !!! namespace added !!!
        namespace='target',
        name="target_spawn_controller_joint_state_broadcaster",
        arguments=["target_diff_controller", "--controller-manager", "/target/controller_manager"],
        output="screen",
    )

    target_joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # !!! namespace added !!!
        namespace='target',
        name="target_spawn_controller_joint_state_broadcaster",
        arguments=["target_joint_broadcaster", "--controller-manager", "/target/controller_manager"],
        output="screen",
    )
    
    # Launch everything!
    return LaunchDescription([
        # SetEnvironmentVariable('GAZEBO_MODEL_PATH', '/home/nusshao/.gazebo/models'),
        rsp,
        gazebo,

        spawn_entity_robot,
        spawn_entity_target,
        
        target_joint_broad_spawner,
        target_diff_drive_spawner,

        joint_broad_spawner,
        diff_drive_spawner,
    ])

