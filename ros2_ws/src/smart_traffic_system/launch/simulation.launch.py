import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_smart_traffic = get_package_share_directory('smart_traffic_system')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_smart_traffic, 'worlds', 'intersection.world')
    bridge_config = os.path.join(pkg_smart_traffic, 'config', 'bridge_config.yaml')
    models_path = os.path.join(pkg_smart_traffic, 'models')
    
    # Try to get gazebo_ros_actor_plugin skins path for DoctorFemaleWalk model
    try:
        pkg_actor_plugin = get_package_share_directory('gazebo_ros_actor_plugin')
        actor_skins_path = os.path.join(pkg_actor_plugin, 'config', 'skins')
    except Exception:
        # Fallback to source directory if package not installed
        actor_skins_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(pkg_smart_traffic))),
            'src', 'gazebo-ros-actor-plugin', 'config', 'skins'
        )
    
    # Combine model paths (Gazebo uses colon-separated paths)
    gz_resource_path = f"{models_path}:{actor_skins_path}"

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_path,
            description='World to load'
        ),
        # Set Gazebo Resource Path (includes both traffic models and actor skins)
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path
        ),
        # Launch Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': [LaunchConfiguration('world'), ' -r']
            }.items(),
        ),
        # ROS 2 <-> Gazebo Bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_bridge'), 'launch', 'ros_gz_bridge.launch.py')
            ),
            launch_arguments={
                'config_file': bridge_config,
                'bridge_name': 'smart_traffic_bridge'
            }.items(),
        ),
        # Traffic Flow Controller
        Node(
            package='smart_traffic_system',
            executable='traffic_flow.py',
            name='traffic_flow_controller',
            output='screen'
        ),
        # Smart Traffic Manager
        Node(
            package='smart_traffic_system',
            executable='smart_traffic_manager.py',
            name='smart_traffic_manager',
            output='screen'
        ),
        # Intersection Zone Monitor
        Node(
            package='smart_traffic_system',
            executable='intersection_zone_monitor.py',
            name='intersection_zone_monitor',
            output='screen'
        ),
    ])
