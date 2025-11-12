import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node,ComposableNodeContainer
def generate_launch_description():
    ld=LaunchDescription()
    param_file_path=os.path.join(get_package_share_directory('rc_navigation'),'config','nav2_params_real.yaml')
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false',description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument('use_composition',default_value='False',description='Use lifecycle nodes'))
    ld.add_action(DeclareLaunchArgument('params_file',default_value=param_file_path,description='Full path to the ROS2 parameters file to use'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='True', description='Automatically startup the nav2 stack'))
    ld.add_action(DeclareLaunchArgument('container_name', default_value='nav2_container', description='组合容器名称'))
    #======调用原始launch文件，传入参数======
    my_packager_share_dir = get_package_share_directory('rc_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = IncludeLaunchDescription(
        ([my_packager_share_dir, '/launch', '/navigation.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'params_file': LaunchConfiguration('params_file')
                          }.items()
    )
    #======================================启动地图服务
    map_path=os.path.join(my_packager_share_dir,'map','court_map_real.yaml')
    # map_path=os.path.join(my_packager_share_dir,'map','empty_map.yaml')
    ld.add_action(DeclareLaunchArgument('map', default_value=map_path, description='Full path to map yaml file to load'))
    map_server_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([my_packager_share_dir,'/launch','/map_server.launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'map': LaunchConfiguration('map')}.items()
    )
    nav2_container = ComposableNodeContainer(
        condition=IfCondition(LaunchConfiguration('use_composition')),
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        parameters=[param_file_path],      #为了给cost_map传参拉的💩,链接为:https://github.com/ros-navigation/navigation2/issues/2147#issuecomment-915890304
    )
    ld.add_action(nav2_bringup_launch)
    ld.add_action(map_server_launch)
    ld.add_action(nav2_container)
    return ld