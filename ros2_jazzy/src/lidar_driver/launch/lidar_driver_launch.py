from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
# from ament_index_python.packages import get_package_share_directory
# 设置环境变量------------------
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # 开启终端的着色(不开启时,Launch启动的节点发送的日志不会根据日志等级改变颜色)
    colorized_output = SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1')

    # 创建节点并且传入声明的参数
    lidar_node = Node(
        package='lidar_driver',
        executable='lidar_node',
        name='ldlidar_publisher_ld14',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD14'},
            {'laser_scan_topic_name': 'scan'},
            {'point_cloud_2d_topic_name': 'pointcloud2d'},
            {'frame_id': 'laser_Link'},
            {'port_name': '/dev/ttyVIRT1'},
            {'serial_baudrate' : 115200},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    return LaunchDescription([
        colorized_output,
        lidar_node
    ])



'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

