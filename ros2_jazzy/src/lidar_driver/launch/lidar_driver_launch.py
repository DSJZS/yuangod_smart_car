from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
# from ament_index_python.packages import get_package_share_directory
# 设置环境变量------------------
from launch.actions import SetEnvironmentVariable

'''
参数说明:
---
- 设置激光扫描方向: 
    1. 设置逆时针扫描，示例: {'laser_scan_dir': True}
    2. 设置顺时针扫描，示例: {'laser_scan_dir': False}
- 角度裁剪设置，屏蔽设定角度范围内的数据:
    1. 启用角度裁剪功能:
        1.1. 启用角度裁剪，示例: {'enable_angle_crop_func': True}
        1.2. 禁用角度裁剪，示例: {'enable_angle_crop_func': False}
    2. 角度裁剪区间设置:
        - 设置在指定角度范围内的距离和强度数据将被归零.
        - 角度范围满足: angle ≥ 'angle_crop_min' 且 angle ≤ 'angle_crop_max'，即区间 [angle_crop_min, angle_crop_max]，单位是度.
            示例:
            {'angle_crop_min': 135.0}
            {'angle_crop_max': 225.0}
            即区间 [135.0, 225.0]，角度单位为度.
'''


def generate_launch_description():
    # 开启终端的着色(不开启时,Launch启动的节点发送的日志不会根据日志等级改变颜色)
    colorized_output = SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1')

    # 参数的声明 laser_Link
    decl_product_name = DeclareLaunchArgument( name="product_name",default_value="LDLiDAR_LD14")
    decl_laser_scan_topic_name = DeclareLaunchArgument( name="laser_scan_topic_name",default_value="scan")
    decl_point_cloud_2d_topic_name = DeclareLaunchArgument( name="point_cloud_2d_topic_name",default_value="pointcloud2d") 
    decl_frame_id = DeclareLaunchArgument( name="frame_id",default_value="base_laser")
    decl_port_name = DeclareLaunchArgument( name="port_name",default_value="/dev/ttyVIRT1")
    decl_serial_baudrate = DeclareLaunchArgument( name="serial_baudrate",default_value="115200")
    decl_laser_scan_dir = DeclareLaunchArgument( name="laser_scan_dir",default_value="True")
    decl_enable_angle_crop_func = DeclareLaunchArgument( name="enable_angle_crop_func",default_value="False")
    decl_angle_crop_min = DeclareLaunchArgument( name="angle_crop_min",default_value="135.0")
    decl_angle_crop_max = DeclareLaunchArgument( name="angle_crop_max",default_value="225.0")

    # 创建节点并且传入声明的参数
    lidar_node = Node(
        package='lidar_driver',
        executable='lidar_node',
        name='ldlidar_publisher_ld14',
        output='screen',
        parameters=[
            {'product_name': LaunchConfiguration("product_name")},
            {'laser_scan_topic_name': LaunchConfiguration("laser_scan_topic_name")},
            {'point_cloud_2d_topic_name': LaunchConfiguration("point_cloud_2d_topic_name")},
            {'frame_id': LaunchConfiguration("frame_id")},
            {'port_name': LaunchConfiguration("port_name")},
            {'serial_baudrate' : LaunchConfiguration("serial_baudrate")},
            {'laser_scan_dir': LaunchConfiguration("laser_scan_dir")},
            {'enable_angle_crop_func': LaunchConfiguration("enable_angle_crop_func")},
            {'angle_crop_min': LaunchConfiguration("angle_crop_min")},
            {'angle_crop_max': LaunchConfiguration("angle_crop_max")}
        ]
    )

    # 创建退出事件, 当 chassis_node节点 退出时输出LOG
    exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action = lidar_node,
            on_exit = [LogInfo(msg = "lidar_node退出!")]
        )
    )

    return LaunchDescription([
        colorized_output,
        decl_product_name,
        decl_laser_scan_topic_name,
        decl_point_cloud_2d_topic_name,
        decl_frame_id,
        decl_port_name,
        decl_serial_baudrate,
        decl_laser_scan_dir,
        decl_enable_angle_crop_func,
        decl_angle_crop_min,
        decl_angle_crop_max,
        lidar_node,
        exit_event
    ])
