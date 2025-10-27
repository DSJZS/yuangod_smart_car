/* 头文件引用 */
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"

/* 用户自定义节点对象 */
class LidarNode: public rclcpp::Node{
public:
    LidarNode():Node("lidar_node_cpp"){
        /* 节点构造函数体 */
        RCLCPP_INFO( this->get_logger(), "lidar_node_cpp 创建成功");
        
    }
};

int main(int argc, char ** argv)
{
    /* 初始化ROS2客户端 */
    rclcpp::init( argc, argv);
    /* 调用spin函数，并传入节点对象的指针 */
    rclcpp::spin(std::make_shared<LidarNode>());
    /* 释放ROS2客户端资源 */
    rclcpp::shutdown();
    return 0;
}