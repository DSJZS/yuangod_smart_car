#!/bin/bash #脚本解释器为shell

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
ros2 launch chassis_driver chassis_node_launch.py imu_frame:=pcb_link serial_device_name:=/dev/ttyVIRT0 &
CHASSIS_DRIVER_PID=$!

# 启动雷达功能包!!!
ros2 launch lidar_driver lidar_driver_launch.py frame_id:=laser_Link angle_offset:=180.0 port_name:=/dev/ttyVIRT1 &
LIDAR_DRIVER_PID=$!

# 等待功能包启动完成(可选)
sleep 1

echo "可视化功能包已启动: PID $YUANGOD_DESCRIPTION_PID"
echo "底盘功能包已启动: PID $CHASSIS_DRIVER_PID"
echo "雷达功能包已启动: PID $LIDAR_DRIVER_PID"
echo -e "${RED}按 Ctrl+C 停止所有程序${RESET}"

cleanup() {
    kill -- -$CHASSIS_DRIVER_PID
    kill -- -$YUANGOD_DESCRIPTION_PID
    kill -- -$LIDAR_DRIVER_PID
    . close_tcp2vsp.sh
}

# 等待中断信号
trap cleanup INT TERM EXIT

wait