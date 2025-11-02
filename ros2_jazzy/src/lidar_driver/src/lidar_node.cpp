/* 头文件引用 */
#include "lidar_driver/ldlidar_ls.h"

/* 功能包作用介绍:
 *  本功能包用于创建激光雷达节点, 功能如下:
 *      1. 通过串口获取激光雷达的数据, 然后处理并发布到对应主题
 *  值得注意的是串口不一定为一个真实的物理串口，可以是通过socat等工具创建的虚拟串口
 *      本项目就使用socat创建了一个绑定TCP端口的虚拟串口
 */

/**
  * @brief 启动乐动机器人官方针对LD14的Demo程序(下述的参数为节点参数)
  *
  * @param product_name                 激光雷达参品的名称， 只可为 “LDLiDAR_LD14” 或者 "LDLiDAR_LD14P", 决定了驱动如何配置与运转
  * @param laser_scan_topic_name        sensor_msgs::msg::LaserScan 类型消息发布的主题的名称
  * @param point_cloud_2d_topic_name    sensor_msgs::msg::PointCloud 类型消息发布的主题的名称(值得注意的是ROS2官方建议换为 PointCloud2)
  *                                         原文为: "THIS MESSAGE IS DEPRECATED AS OF FOXY, use PointCloud2 instead"
  * @param frame_id                     激光雷达的坐标系
  * @param port_name                    串口设备名称
  * @param serial_baudrate              串口设备波特率， LD14 为 115200 ； LD14P 为 230400. 手册上规定的数值,不建议更改
  * @param laser_scan_dir               设置激光扫描方向, True 为逆时针 ; False 为顺时针
  * @param enable_angle_crop_func       角度裁剪功能使能, True 为启用 ; False 为关闭. 启用后在[angle_crop_min, angle_crop_max]区间内的距离和强度数据将被归零
  * @param angle_crop_min               角度裁剪功能指定区间的最小值， enable_angle_crop_func 为 False 时无用, 单位为度
  * @param angle_crop_max               角度裁剪功能指定区间的最大值， enable_angle_crop_func 为 False 时无用, 单位为度
  * @param angle_offset                 角度的偏置, 偏置方向固定为顺时针, 单位为度
  * @return
  *      - status: 为 0 表示正常退出, 否则表示是因为错误退出
  */
int main(int argc, char ** argv)
{
    /* 将程序控制权交给官方的Demo 
     * 顺便说一句, 这个Demo默认激光雷达的扫描频率为 6Hz, 这是因为 LD14 和 LD14P 的默认出场配置为 6Hz
     * 如果你雷达的 PWM 引脚没有接地或悬空, 希望自行调速, 请自行完成对应的功能
     * 配置扫描频率的参数在本文件夹下的 ldlidar_ls.cpp 之中
     * 通过查找 `rclcpp::WallRate r(6);` 即可快速找到对应的语句
     */
    return ld_lidar_startup( argc, argv, "lidar_node_cpp");
}
