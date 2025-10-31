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

def generate_launch_description():
    # 开启终端的着色(不开启时,Launch启动的节点发送的日志不会根据日志等级改变颜色)
    colorized_output = SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1')

    # 参数的声明
    decl_serial_device_name = DeclareLaunchArgument( name="serial_device_name",default_value="/dev/ttyVIRT0")
    decl_baud_rate = DeclareLaunchArgument( name="baud_rate",default_value="115200")
    decl_frame_head = DeclareLaunchArgument( name="frame_head",default_value="0xAA") # 如果错误可以换成十进制也就是 "170"
    decl_twist_topic_name = DeclareLaunchArgument( name="twist_topic_name",default_value="cmd_vel")
    decl_odom_topic_name = DeclareLaunchArgument( name="odom_topic_name",default_value="odom")
    decl_imu_topic_name = DeclareLaunchArgument( name="imu_topic_name",default_value="imu")
    decl_odom_frame = DeclareLaunchArgument( name="odom_frame",default_value="odom")
    # decl_imu_frame = DeclareLaunchArgument( name="imu_frame",default_value="imu_link")
    decl_imu_frame = DeclareLaunchArgument( name="imu_frame",default_value="base_link")
    decl_base_frame = DeclareLaunchArgument( name="base_frame",default_value="base_link")

    # 创建节点并且传入声明的参数
    chassis_node = Node(
        package="chassis_driver",
        executable="chassis_node",
        parameters=[{
            "serial_device_name": LaunchConfiguration("serial_device_name"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "frame_head": LaunchConfiguration("frame_head"),
            "twist_topic_name": LaunchConfiguration("twist_topic_name"),
            "odom_topic_name": LaunchConfiguration("odom_topic_name"),
            "imu_topic_name": LaunchConfiguration("imu_topic_name"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "imu_frame": LaunchConfiguration("imu_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
        }]
    )

    # 创建退出事件, 当 chassis_node节点 退出时输出LOG
    exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action = chassis_node,
            on_exit = [LogInfo(msg = "chassis_node退出!")]
        )
    )

    return LaunchDescription([
        colorized_output,
        decl_serial_device_name,
        decl_baud_rate,
        decl_frame_head,
        decl_twist_topic_name,
        decl_odom_topic_name,
        decl_imu_topic_name,
        decl_odom_frame,
        decl_imu_frame,
        decl_base_frame,
        chassis_node,
        exit_event,
    ])