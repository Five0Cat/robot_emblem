# launch/show_unit.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
# 如果你确实需要改 LD_LIBRARY_PATH，再解开下面两行：
# from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # 包位置
    robot_pkg = FindPackageShare('robot_emblem')
    desc_pkg  = FindPackageShare('robot_emblem_description')

    # 资源路径（用 PathJoinSubstitution，不用 os.path.join）
    map_yaml  = PathJoinSubstitution([robot_pkg, 'maps', 'demo_map.yaml'])
    rviz_cfg  = PathJoinSubstitution([robot_pkg, 'rviz', 'demo.rviz'])
    urdf_xacro = PathJoinSubstitution([desc_pkg,  'urdf', 'fe_unit.urdf.xacro'])
    enemy_xacro = PathJoinSubstitution([desc_pkg, 'urdf', 'enemy.urdf.xacro'])


    enemy_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='enemy_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', enemy_xacro]),
            'frame_prefix': 'enemy/'   # avoid crash with player unit
        }]
    )
    # robot_state_publisher：xacro → robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_xacro])
        }]
    )

    # 你的两个节点
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

    # 可选：启动 RViz 并加载配置
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_cfg],
    # )

    # 如果真的需要手动改 LD_LIBRARY_PATH（一般不需要），可以启用：
    # safe_ld = ":".join([
    #   "/opt/ros/humble/opt/rviz_ogre_vendor/lib",
    #   "/opt/ros/humble/lib",
    #   "/opt/ros/humble/lib/x86_64-linux-gnu",
    #   "/usr/lib/x86_64-linux-gnu",
    #   "/usr/lib",
    # ])
    # env_ld = SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=safe_ld)

    return LaunchDescription([
        # env_ld,  # 如需设置再放开
        rsp,
        map_loader,
        unit_mgr,
        enemy_rsp
        # rviz,
    ])
