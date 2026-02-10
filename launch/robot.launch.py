import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def _make_robot_group(*, ns: str, entity_name: str,
                      robot_description: str,
                      gz_bridge_params_path: str,
                      x, y, z, roll, pitch, yaw):
    """
    Create a namespaced robot:
      - Spawns a unique Gazebo entity name
      - Runs robot_state_publisher in a namespace with a TF frame prefix
      - Runs a ros_gz_bridge parameter_bridge in the same namespace
    """
    return GroupAction([
        PushRosNamespace(ns),

        # Spawn in Gazebo (entity name MUST be unique)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', entity_name,
                '-string', robot_description,
                '-x', x,
                '-y', y,
                '-z', z,
                '-R', roll,
                '-P', pitch,
                '-Y', yaw,
                '-allow_renaming', 'false'
            ],
            output='screen',
        ),

        # Publish TF/robot state; frame_prefix prevents TF collisions between robots
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': True,
                    # This prefixes every frame id, e.g. "base_link" -> "robot1/base_link"
                    'frame_prefix': f'{ns}/',
                }
            ],
            output='screen'
        ),

        # Bridge in the same namespace so topics become:
        #   /robot1/cmd_vel, /robot1/odom, ...
        #   /robot2/cmd_vel, /robot2/odom, ...
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args', '-p',
                f'config_file:={gz_bridge_params_path}'
            ],
            output='screen'
        ),
    ])


def generate_launch_description():
    package_name = "gazebo_differential_drive_robot"

    # Gazebo world
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Specify the world file for Gazebo (e.g., empty.sdf)'
    )
    world_file = LaunchConfiguration('world')

    # Robot 1 pose args
    x1_arg = DeclareLaunchArgument('x1', default_value='0.0', description='Robot1 initial X')
    y1_arg = DeclareLaunchArgument('y1', default_value='0.0', description='Robot1 initial Y')
    z1_arg = DeclareLaunchArgument('z1', default_value='0.5', description='Robot1 initial Z')
    R1_arg = DeclareLaunchArgument('R1', default_value='0.0', description='Robot1 initial Roll')
    P1_arg = DeclareLaunchArgument('P1', default_value='0.0', description='Robot1 initial Pitch')
    Y1_arg = DeclareLaunchArgument('Y1', default_value='0.0', description='Robot1 initial Yaw')

    # Robot 2 pose args (defaults offset so it doesn’t spawn on top of robot1)
    x2_arg = DeclareLaunchArgument('x2', default_value='4.0', description='Robot2 initial X')
    y2_arg = DeclareLaunchArgument('y2', default_value='0.0', description='Robot2 initial Y')
    z2_arg = DeclareLaunchArgument('z2', default_value='0.5', description='Robot2 initial Z')
    R2_arg = DeclareLaunchArgument('R2', default_value='0.0', description='Robot2 initial Roll')
    P2_arg = DeclareLaunchArgument('P2', default_value='0.0', description='Robot2 initial Pitch')
    Y2_arg = DeclareLaunchArgument('Y2', default_value='3.14', description='Robot2 initial Yaw')

    x1 = LaunchConfiguration('x1'); y1 = LaunchConfiguration('y1'); z1 = LaunchConfiguration('z1')
    R1 = LaunchConfiguration('R1'); P1 = LaunchConfiguration('P1'); Y1 = LaunchConfiguration('Y1')

    x2 = LaunchConfiguration('x2'); y2 = LaunchConfiguration('y2'); z2 = LaunchConfiguration('z2')
    R2 = LaunchConfiguration('R2'); P2 = LaunchConfiguration('P2'); Y2 = LaunchConfiguration('Y2')

    # Paths
    robot_model_path = os.path.join(
        get_package_share_directory(package_name),
        'model',
        'robot.xacro'
    )
    gz_bridge_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge.yaml'
    )

    # URDF from Xacro (same robot for both)
    robot_description = xacro.process_file(robot_model_path).toxml()

    # Gazebo launch include
    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )
    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={
            'gz_args': [f'-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Two robots, namespaced
    robot1 = _make_robot_group(
        ns='robot1',
        entity_name='differential_drive_robot_1',
        robot_description=robot_description,
        gz_bridge_params_path=gz_bridge_params_path,
        x=x1, y=y1, z=z1, roll=R1, pitch=P1, yaw=Y1
    )

    robot2 = _make_robot_group(
        ns='robot2',
        entity_name='differential_drive_robot_2',
        robot_description=robot_description,
        gz_bridge_params_path=gz_bridge_params_path,
        x=x2, y=y2, z=z2, roll=R2, pitch=P2, yaw=Y2
    )

    return LaunchDescription([
        world_arg,
        gazebo_launch,

        x1_arg, y1_arg, z1_arg, R1_arg, P1_arg, Y1_arg,
        x2_arg, y2_arg, z2_arg, R2_arg, P2_arg, Y2_arg,

        robot1,
        robot2,
    ])
