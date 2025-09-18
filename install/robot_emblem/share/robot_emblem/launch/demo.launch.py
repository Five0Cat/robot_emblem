from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_emblem')
    map_yaml = os.path.join(pkg_share, 'maps', 'demo_map.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'demo.rviz')  # 可选 rviz 配置文件
    return LaunchDescription([
        Node(
            package='robot_emblem',
            executable='map_loader_node',
            name='map_loader_node',
            output='screen',
            parameters=[{'map_yaml': map_yaml}],
        ),
        Node(
            package='robot_emblem',
            executable='unit_manager_node',
            name='unit_manager_node',
            output='screen',
            parameters=[
                {'map_yaml': map_yaml},
                {'unit_ix': 1},
                {'unit_iy': 1},
            ],
        ),
    #    Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config],   # 如果没有 rviz_config 文件，可以先去掉这一行
    #     additional_env={
    #         'QT_QPA_PLATFORM': 'xcb',     # 强制用 XCB，不走 Wayland
          
    #     },
    # )
    ])
