import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
import os

from launch.actions  import IncludeLaunchDescription 
from launch.launch_description_sources  import PythonLaunchDescriptionSource 
from ament_index_python.packages  import get_package_share_directory 



def generate_launch_description():

    urdf_package_path = get_package_share_directory('server_python')
    # default_urdf_path = os.path.join(urdf_package_path,'urdf','first_robot.urdf')
    default_rviz_config_path = os.path.join(urdf_package_path,'parking_rviz','display_robot_model.rviz')
    map_yaml_path = os.path.join(urdf_package_path,'map','parking_map.yaml')

    node_tf_broadcaster = launch_ros.actions.Node(
        package='server_python',
        executable='base_tf_map',
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )
    
    map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_path}]
    )

    scout_des_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('scout_description'),
            '/launch/scout_base_description.launch.py' 
        ]),
    )

    parking_server = launch_ros.actions.Node(
        package='server_python',
        executable='parking_server',
    )

    return launch.LaunchDescription([
        node_tf_broadcaster,
        map_server,
        rviz_node,
        scout_des_launch,
        parking_server,

    ])