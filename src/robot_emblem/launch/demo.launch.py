# launch/show_unit.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_pkg = FindPackageShare('robot_emblem')
    desc_pkg  = FindPackageShare('robot_emblem_description')

    map_yaml   = PathJoinSubstitution([robot_pkg, 'maps', 'demo_map.yaml'])
    rviz_cfg   = PathJoinSubstitution([robot_pkg, 'rviz', 'demo.rviz'])
    ally_xacro = PathJoinSubstitution([desc_pkg, 'urdf', 'fe_unit.urdf.xacro'])
    enemy_xacro= PathJoinSubstitution([desc_pkg, 'urdf', 'enemy.urdf.xacro'])

    # 玩家模型：/robot_description（默认）
    ally_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ally_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', ally_xacro]),
            # 关键：给玩家也加前缀，与 C++ 的 "ally/base_link" 对齐
            'frame_prefix': 'ally/'
        }]
    )

    # 敌人模型：重映射 robot_description -> /enemy/robot_description
    enemy_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='enemy_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', enemy_xacro]),  # 注意有空格
            'frame_prefix': 'enemy/'
        }]
        # 不要 remappings；参数不是话题
    )
    map_loader = Node(
        package='robot_emblem',
        executable='map_loader_node',
        name='map_loader_node',
        output='screen',
        parameters=[{'map_yaml': map_yaml}],
    )

    unit_mgr = Node(
        package='robot_emblem',
        executable='unit_manager_node',
        name='unit_manager_node',
        output='screen',
        parameters=[{'map_yaml': map_yaml, 'unit_ix': 1, 'unit_iy': 1}],
    )

    # 可选：直接带上 RViz
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_cfg],
    # )

    return LaunchDescription([
        ally_rsp,
        enemy_rsp,
        map_loader,
        unit_mgr,
        # rviz,
    ])
