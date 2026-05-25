#!/usr/bin/env python3
"""
倒立摆起摆仿真 Launch 文件

启动顺序:
1. 加载 URDF 模型
2. 启动 Gazebo 仿真
3. 启动 robot_state_publisher
4. 启动控制器管理器
5. 启动起摆控制器节点
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 包路径
    pkg_share = FindPackageShare('pendulum_swing')
    
    # URDF 文件路径
    urdf_file = 'pendulum_swing.urdf.xacro'
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', urdf_file])
    
    # 控制器配置文件路径
    controller_config_file = PathJoinSubstitution([pkg_share, 'config', 'pendulum_controllers.yaml'])
    
    # 控制器列表
    controllers = ['joint_state_broadcaster', 'cart_effort_controller']
    
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    
    # 环境变量（纯 CPU 软件渲染，兼容性最强）
    set_ign_render_engine = SetEnvironmentVariable('IGN_RENDER_ENGINE', 'ogre')
    set_ogre_rtti = SetEnvironmentVariable('OGRE_RTTI_MODE', '1')
    set_software_gl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    set_ign_plugin_path = AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '/opt/ros/humble/lib')
    set_ld_library_path = AppendEnvironmentVariable('LD_LIBRARY_PATH', '/opt/ros/humble/lib')
    
    # 1. Robot State Publisher - 发布机器人状态
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command([
                'xacro', ' ', urdf_path
            ]), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    
    # 2. Gazebo (Ignition Fortress) - 启动仿真环境
    # 使用系统自带的 empty.sdf
    # 不指定 --render-engine，靠环境变量 IGN_RENDER_ENGINE
    # -r: 实时运行
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )
    
    # 3. Spawn Robot - 将机器人放入 Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'pendulum_swing',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        output='screen',
    )
    
    # ROS-Gazebo Bridge - 同步时间戳
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
    )
    
    # 4. Controllers - 使用 controller_manager spawner
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'],
    )
    
    cart_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cart_effort_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'],
    )
    
    # 5. Swing Up Controller Node - 起摆控制器
    swing_up_controller = Node(
        package='pendulum_swing',
        executable='swing_up_controller',
        name='swing_up_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    
    # 事件处理：启动顺序
    # 1. robot_state_publisher 启动后启动 Gazebo
    delay_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[gazebo],
        )
    )
    
    # 2. Gazebo 启动后延迟生成实体
    delay_spawn = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[TimerAction(period=2.0, actions=[spawn_entity])],
        )
    )
    
    # 3. 实体生成后启动 bridge 和控制器
    delay_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[bridge, joint_state_broadcaster],
        )
    )
    
    # 4. joint_state_broadcaster 启动后启动 effort controller
    delay_effort_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[TimerAction(period=1.0, actions=[cart_effort_controller])],
        )
    )
    
    # 5. effort controller 启动后启动 swing_up_controller
    start_swing_up_controller_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=cart_effort_controller,
            on_exit=[TimerAction(period=1.0, actions=[swing_up_controller])],
        )
    )
    
    return LaunchDescription([
        # 环境变量（必须在最前面）
        set_ign_render_engine,
        set_ogre_rtti,
        set_software_gl,
        set_ign_plugin_path,
        set_ld_library_path,
        # 参数声明
        declare_use_sim_time,
        # 启动序列
        robot_state_publisher,
        delay_gazebo,
        delay_spawn,
        delay_controllers,
        delay_effort_controller,
        start_swing_up_controller_handler,
    ])