import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
import os

from launch.actions  import IncludeLaunchDescription 
from launch.launch_description_sources  import PythonLaunchDescriptionSource 
from ament_index_python.packages  import get_package_share_directory 



def generate_launch_description():

    urdf_package_path = get_package_share_directory('autoparking_python')
    default_rviz_config_path = os.path.join(urdf_package_path,'parking_rviz','display_robot_model.rviz')
    map_yaml_path = os.path.join(urdf_package_path,'map','parking_map.yaml')


    action_node_tf_broadcaster = launch_ros.actions.Node(
        package='autoparking_python',
        executable='base_tf_map',
    )

    action_rviz_node = launch_ros.actions.Node(
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

    N100_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('fdlink_ahrs'),
            '/launch/ahrs_driver.launch.py' 
        ]),
    )
    
    scout_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('scout_base'),
            '/launch/scout_base.launch.py' 
        ]),
    )

    scout_des_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('scout_description'),
            '/launch/scout_base_description.launch.py' 
        ]),
    )
    car1_node = launch_ros.actions.Node(
        package='autoparking_python',
        executable='car1_node',
    )
    car1_test = launch_ros.actions.Node(
        package='autoparking_python',
        executable='car1_test',
    )
    
    car2_node = launch_ros.actions.Node(
        package='autoparking_python',
        executable='car2_node',
    )
    uwb_node = launch_ros.actions.Node(
        package='autoparking_python',
        executable='uwb',
    )
    parking_server = launch_ros.actions.Node(
        package='autoparking_python',
        executable='parking_server',
    )

    odom_node = launch_ros.actions.Node(
        package='autoparking_python',
        executable='odom',
    )

    return launch.LaunchDescription([
        N100_launch,
        scout_launch,
        # car1_node,
        # car2_node,
        # parking_server,
        car1_test,
        uwb_node,     # TODO: DWM1000的ge 用这个
        odom_node,

    ])