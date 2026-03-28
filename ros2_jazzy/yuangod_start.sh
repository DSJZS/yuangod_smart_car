#!/bin/bash

set -- $(getopt -qu s "$@")

continuing_mapping_mode='n'

while [ -n "$1" ]; do
    opt="$1"
    case "$opt" in
        -s) echo "持续建图模式"
            continuing_mapping_mode='y' ;;
        --) shift
            break ;;
        *) echo "unknow option: $opt" ;;
    esac
    shift
done

# 重置颜色
RESET='\033[0m'

# 基础颜色
RED='\033[0;31m'

# 遇到错误立刻退出，安全措施
# set -e

# 创建TCP转虚拟串口
. create_tcp2vsp.sh

# 等待虚拟串口启动完成(可选,建议延时)
sleep 1

# ROS2配置
# 注意后加载的工作空间配置文件优先级更高，但是ROS2系统的setup.bash无论前后加载，优先级总是最低
. /opt/ros/jazzy/setup.bash
. install/setup.bash

# 可视化功能包启动!!!
ros2 launch yuangod_description yuangod_description_launch.py &
YUANGOD_DESCRIPTION_PID=$!

# 启动底盘功能包!!!
ros2 launch chassis_driver chassis_node_launch.py odom_topic_name:=yuangod/odom imu_topic_name:=yuangod/imu imu_frame:=pcb_link serial_device_name:=/dev/ttyVIRT0 &
CHASSIS_DRIVER_PID=$!

# 等待底盘功能包启动完成
# 因为底盘功能包负责启动 robot_localization 节点，而SLAM和导航功能包都依赖该节点
# 不等待启动完成。肯呢个出现 odom->base_link TF 变换丢失的问题
echo "Wait for chassis_driver to start..."
sleep 5

# 启动雷达功能包!!!
ros2 launch lidar_driver lidar_driver_launch.py frame_id:=laser_Link angle_offset:=180.0 port_name:=/dev/ttyVIRT1 &
LIDAR_DRIVER_PID=$!

# 启动SLAM功能包!!!
ros2 launch slam_launch online_async_launch.py use_sim_time:=False &
SLAM_PID=$!

# 启动导航功能包!!!
ros2 launch nav2_launch navigation_launch.py use_sim_time:=False &
NAV2_PID=$!

echo "可视化功能包已启动: PID $YUANGOD_DESCRIPTION_PID"
echo "底盘功能包已启动: PID $CHASSIS_DRIVER_PID"
echo "雷达功能包已启动: PID $LIDAR_DRIVER_PID"
echo "SLAM功能包已启动: PID $SLAM_PID"
echo "导航功能包已启动: PID $NAV2_PID"
echo -e "${RED}按 Ctrl+C 停止所有程序${RESET}"

cleanup() {
    kill -- -$CHASSIS_DRIVER_PID
    kill -- -$YUANGOD_DESCRIPTION_PID
    kill -- -$LIDAR_DRIVER_PID
    kill -- -$SLAM_PID
    kill -- -$NAV2_PID
    echo "所有功能包已停止"
    # 关闭TCP转虚拟串口
    . close_tcp2vsp.sh
}

# 等待中断信号
trap cleanup INT TERM EXIT

wait