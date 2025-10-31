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
        package="lidar_driver",
        executable="lidar_node",
    )
    return LaunchDescription([
        colorized_output,
        lidar_node
    ])