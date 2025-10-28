/* 头文件引用 */
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
/* 第三方库引用 */
#include "lwrb/lwrb.h"  
#include "simple_frame/simple_frame.hpp"

using cya::hal::protocol::Simple_Frame;

/* 功能包作用介绍:
 *  本功能包用于创建底盘节点, 功能如下:
 *      1. 通过串口获取底盘各个传感器的数据并发布到对应主题
 *      2. 接收来自于 `/cmd_vel` 主题的速度消息，并通过串口转发到底盘用于控制机器人移动
 *  值得注意的是串口不一定为一个真实的物理串口，可以是通过socat等工具创建的虚拟串口
 *      本项目就使用socat创建了一个绑定TCP端口的虚拟串口
 */

/* 底盘节点对象类型 */
class ChassisNode: public rclcpp::Node{
public:
    ChassisNode( const std::string& node_name, const std::string& serial_device_name, uint32_t baud_rate, uint8_t packet_head);
    void read_sensors_data( std::vector<uint8_t> &data, const size_t &size);
    void send_command( const geometry_msgs::msg::Twist& msg);
private:
    std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::shared_ptr<drivers::common::IoContext> io_context_;
    std::unique_ptr<std::vector<uint8_t>> transmit_data_buffer_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    /* 第三方库提供的 环形缓冲区 */
    lwrb_t ring_buffer_;
    uint8_t rb_data_[512];  //  这个512可以根据实际情况进行调整,个人测试已经十分的够用了
    /* 第三方库提供的 简单帧处理对象 */
    Simple_Frame sfp_;      //  上述的512如果更改了要顺便更改 Simple_Frame::kParserBufferLength 的值
};

int main(int argc, char ** argv)
{
    /* 初始化ROS2客户端 */
    rclcpp::init( argc, argv);

    /* 创建对象并等待回调函数 */
    auto chassis_node = std::make_shared<ChassisNode>( "chassis_node_cpp", "/dev/ttyVIRT0", 115200, 0xAA);
    rclcpp::spin(chassis_node);

    /* 释放ROS2客户端资源 */
    rclcpp::shutdown();
    return 0;
}

/**
  * @brief 底盘节点构造函数
  *
  * @param node_name 节点名称
  * @param serial_device_name 串口设备名称
  * @param baud_rate 串口波特率
  * @return
  *      - Value: Text
  */
ChassisNode::ChassisNode(const std::string& node_name, const std::string& serial_device_name, uint32_t baud_rate, uint8_t packet_head)
    : Node(node_name), sfp_(packet_head)
{
    /* 节点构造函数体 */
    const char* c_node_name = node_name.c_str();
    RCLCPP_INFO( this->get_logger(), "%s 节点创建成功, 开始初始化节点", c_node_name);

    /* 创建串口配置对象 */
    drivers::serial_driver::SerialPortConfig serial_config(
        baud_rate,                                     //  波特率
        drivers::serial_driver::FlowControl::NONE,  //  不开启流控制
        drivers::serial_driver::Parity::NONE,       //  不开启奇偶校验
        drivers::serial_driver::StopBits::ONE);     //  停止位设为1
    /* 初始化串口 */
    try {
        this->io_context_ = std::make_shared<drivers::common::IoContext>(1);
        /* 初始化 serial_driver_ */
        this->serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
        this->serial_driver_->init_port(serial_device_name, serial_config);
        this->serial_driver_->port()->open();
        
        RCLCPP_INFO(this->get_logger(), "串口初始化成功");
        RCLCPP_INFO(this->get_logger(), "使用设备为: %s", this->serial_driver_->port().get()->device_name().c_str());
        RCLCPP_INFO(this->get_logger(), "波特率为: %d", serial_config.get_baud_rate());
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "串口初始化失败, 原因为 %s", ex.what());
        return;
    }

    /* 创建循环缓冲区用于处理字节流数据 */
    lwrb_init( &(this->ring_buffer_), this->rb_data_, sizeof(this->rb_data_)); /* Initialize buffer */
    /* 创建发送缓存区 */
    this->transmit_data_buffer_ = std::make_unique<std::vector<uint8_t>>(1024);
    /* 设置串口异步接收回调 */
    this->serial_driver_->port()->async_receive( std::bind( &ChassisNode::read_sensors_data, this, std::placeholders::_1, std::placeholders::_2));
    /* 订阅速度话题 */
    this->subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind( &ChassisNode::send_command,this,std::placeholders::_1));
}
 
/**
  * @brief 从串口读取底盘上各个传感器的数据并发布到对应主题
  */
void ChassisNode::read_sensors_data( std::vector<uint8_t> &data, const size_t &size)
{
    /* debug */
        // std::string received_message(data.begin(), data.end());
        // RCLCPP_INFO(this->get_logger(), "(%ld bytes Received): %s", size, received_message.c_str());
    /* debug */
    uint8_t command[512] = {0};
    uint16_t command_size = 0;
    lwrb_write( &(this->ring_buffer_), data.data(),size);

    if( this->sfp_.get_command( &(this->ring_buffer_), command, &command_size) ) {
        RCLCPP_INFO(this->get_logger(), "(%u bytes Received): %s", command_size, "test");
    }
}

/**
  * @brief 订阅速度主题的回调函数, 用于通过串口转发速度控制消息到底盘
  *
  * @param msg 运动速度控制消息
  */
void ChassisNode::send_command( const geometry_msgs::msg::Twist& msg)
{
    /* debug */
        RCLCPP_INFO( this->get_logger(), "linear x = %f, y = %f , z = %f", msg.linear.x,msg.linear.y,msg.linear.z);
        RCLCPP_INFO( this->get_logger(), "angular x = %f, y = %f , z = %f", msg.angular.x,msg.angular.y,msg.angular.z);
    /* debug */

    /*

    auto port = this->serial_driver_->port();
 
    try {
      size_t bytes_transmit_size = port->send( *(this->transmit_data_buffer_) );
      RCLCPP_INFO( this->get_logger(), "Transmitted: %s (%ld bytes)", transmitted_message.c_str(), bytes_transmit_size);
    } catch(const std::exception &ex) {
      RCLCPP_ERROR( this->get_logger(), "串口传输发生错误 %s",ex.what());
    }

    */
}