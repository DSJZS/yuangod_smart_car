/* 头文件引用 */
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <string>

/* 用户自定义节点对象 */
class ChassisNode: public rclcpp::Node{
public:
    ChassisNode():Node("chassis_node_cpp"){
        /* 节点构造函数体 */
        RCLCPP_INFO( this->get_logger(), "chassis_node_cpp 创建成功");
        
        /* 串口设备名(根据实际设备调整) */
        const std::string serial_device_name = "/dev/ttyVIRT0";
        /* 创建串口配置对象 */
        drivers::serial_driver::SerialPortConfig serial_config(
            115200,                                     //  波特率
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
            
            RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
            RCLCPP_INFO(this->get_logger(), "Using device: %s", this->serial_driver_->port().get()->device_name().c_str());
            RCLCPP_INFO(this->get_logger(), "Baud_rate: %d", serial_config.get_baud_rate());
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
            return;
        }


    }
private:
    std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::shared_ptr<drivers::common::IoContext> io_context_;
};

int main(int argc, char ** argv)
{
    /* 初始化ROS2客户端 */
    rclcpp::init( argc, argv);
    /* 调用spin函数，并传入节点对象的指针 */
    rclcpp::spin(std::make_shared<ChassisNode>());
    /* 释放ROS2客户端资源 */
    rclcpp::shutdown();
    return 0;
}