/* 头文件引用 */
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <string>
#include <array>
#include <algorithm>
/* 第三方库引用 */
#include "lwrb/lwrb.h"  
#include "simple_frame/simple_frame.hpp"
#include "mahony_filter/mahony_filter.hpp"

/* 功能包作用介绍:
 *  本功能包用于创建底盘节点, 功能如下:
 *      1. 通过串口获取底盘各个传感器的数据并发布到对应主题
 *      2. 接收来自于 `/cmd_vel` 主题的速度消息，并通过串口转发到底盘用于控制机器人移动
 *  值得注意的是串口不一定为一个真实的物理串口，可以是通过socat等工具创建的虚拟串口
 *      本项目就使用socat创建了一个绑定TCP端口的虚拟串口
 */

using cya::hal::protocol::Simple_Frame;

/* 协方差 */
static std::array<double ,36> odom_pose_covariance_dynamic = {
    1e-3,   0,      0,      0,      0,      0, 
    0,      1e-3,   0,      0,      0,      0,
    0,      0,      1e6,    0,      0,      0,
    0,      0,      0,      1e6,    0,      0,
    0,      0,      0,      0,      1e6,    0,
    0,      0,      0,      0,      0,      1e3     };
static std::array<double ,36> odom_pose_covariance_static  = {
    1e-9,   0,      0,      0,      0,      0, 
    0,      1e-9,   0,      0,      0,      0,
    0,      0,      1e6,    0,      0,      0,
    0,      0,      0,      1e6,    0,      0,
    0,      0,      0,      0,      1e6,    0,
    0,      0,      0,      0,      0,      1e-9    };
static std::array<double ,36> odom_twist_covariance_dynamic  = {
    1e-3,   0,      0,      0,      0,      0, 
    0,      1e-3,   0,      0,      0,      0,
    0,      0,      1e6,    0,      0,      0,
    0,      0,      0,      1e6,    0,      0,
    0,      0,      0,      0,      1e6,    0,
    0,      0,      0,      0,      0,      1e3     };
static std::array<double ,36> odom_twist_covariance_static = {
    1e-9,   0,      0,      0,      0,      0, 
    0,      1e-9,   0,      0,      0,      0,
    0,      0,      1e6,    0,      0,      0,
    0,      0,      0,      1e6,    0,      0,
    0,      0,      0,      0,      1e6,    0,
    0,      0,      0,      0,      0,      1e-9    };

/* 底盘节点对象类型 */
class ChassisNode: public rclcpp::Node{
public:
    ChassisNode( const std::string& node_name);
    void read_sensors_data( std::vector<uint8_t> &data, const size_t &size);
    void publish_sensors_data( uint8_t* data, uint16_t size);
    void send_command( const geometry_msgs::msg::Twist& msg);
    bool is_static( float linear_x, float linear_y, float angular_z);
private:
    std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::shared_ptr<drivers::common::IoContext> io_context_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publish_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publish_;

    std::string odom_frame_;
    std::string imu_frame_;
    std::string base_frame_;

    /* 底盘需要记录的位移与姿态 */
    float x_;
    float y_;
    float yaw_;
    /* 时间戳记录 */
    rclcpp::Time current_time_;
    rclcpp::Time last_time_;

    /* 第三方库提供的 环形缓冲区 */
    lwrb_t ring_buffer_;
    uint8_t rb_data_[512];                      //  这个512可以根据实际情况进行调整,个人测试已经十分的够用了
    /* 第三方库提供的 简单帧处理对象 */
    std::unique_ptr<Simple_Frame> sfp_;         //  上述的512如果更改了要顺便更改 Simple_Frame::kParserBufferLength 的值
};

int main(int argc, char ** argv)
{
    /* 初始化ROS2客户端 */
    rclcpp::init( argc, argv);
    
    /* 创建对象并等待回调函数 */
    auto chassis_node = std::make_shared<ChassisNode>(  "chassis_node_cpp" );
    rclcpp::spin(chassis_node);

    /* 释放ROS2客户端资源 */
    rclcpp::shutdown();
    return 0;
}

/**
  * @brief 底盘节点构造函数(下述的参数为节点参数)
  *
  * @param node_name                节点名称
  * @param serial_device_name       串口设备名称
  * @param baud_rate                串口波特率
  * @param frame_head               串口传输的数据帧的帧头
  * @param twist_topic_name         订阅的速度主题
  * @param odom_topic_name          发布的里程计主题
  * @param imu_topic_name           发布的IMU主题
  * @param odom_frame               里程计坐标系名称
  * @param base_frame               基底坐标系名称
  */
ChassisNode::ChassisNode(const std::string& node_name )
    : Node(node_name), x_(0), y_(0), yaw_(0)
{
    /* 节点构造函数体 */
    const char* c_node_name = node_name.c_str();
    RCLCPP_INFO( this->get_logger(), "%s 节点创建成功, 开始初始化节点", c_node_name);

    /* 声明参数并设置默认值 */
    this->declare_parameter("serial_device_name", "/dev/ttyVIRT0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("frame_head", 0xAA);
    this->declare_parameter("twist_topic_name", "cmd_vel");
    this->declare_parameter("odom_topic_name", "odom");
    this->declare_parameter("imu_topic_name", "imu");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("imu_frame", "imu_link");
    // this->declare_parameter("imu_frame", "base_link");
    this->declare_parameter("base_frame", "base_link");
    /* 读取参数 */
    rclcpp::Parameter serial_device_name = this->get_parameter("serial_device_name");
    rclcpp::Parameter baud_rate = this->get_parameter("baud_rate");
    rclcpp::Parameter frame_head = this->get_parameter("frame_head");
    rclcpp::Parameter twist_topic_name = this->get_parameter("twist_topic_name");
    rclcpp::Parameter odom_topic_name = this->get_parameter("odom_topic_name");
    rclcpp::Parameter imu_topic_name = this->get_parameter("imu_topic_name");
    this->get_parameter("odom_frame", this->odom_frame_);
    this->get_parameter("imu_frame", this->imu_frame_);
    this->get_parameter("base_frame", this->base_frame_);
    this->sfp_ = std::make_unique<Simple_Frame>(frame_head.as_int());
    /* 通过日志输出一些主要参数到终端 */
    RCLCPP_INFO(this->get_logger(), "基底坐标系: %s", this->base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "里程计坐标系: %s", this->odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "IMU坐标系: %s", this->imu_frame_.c_str());
    
    /* 创建串口配置对象 */
    drivers::serial_driver::SerialPortConfig serial_config(
        baud_rate.as_int(),                         //  波特率
        drivers::serial_driver::FlowControl::NONE,  //  不开启流控制
        drivers::serial_driver::Parity::NONE,       //  不开启奇偶校验
        drivers::serial_driver::StopBits::ONE);     //  停止位设为1
    /* 初始化串口 */
    try {
        this->io_context_ = std::make_shared<drivers::common::IoContext>(1);
        /* 初始化 serial_driver_ */
        this->serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
        this->serial_driver_->init_port( serial_device_name.as_string() , serial_config);
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

    /* 初始化时间戳 */
    this->current_time_ = this->now();
    this->last_time_ = this->current_time_;
    /* 速度话题订阅 */
    this->twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(twist_topic_name.as_string(), 1, std::bind( &ChassisNode::send_command,this,std::placeholders::_1));
    /* 里程计话题发布 */
    this->odom_publish_ = this->create_publisher<nav_msgs::msg::Odometry>( odom_topic_name.as_string(), 1);
    /* Imu传感器话题发布 */
    this->imu_publish_ = this->create_publisher<sensor_msgs::msg::Imu>( imu_topic_name.as_string(), 1);

    /* 设置串口异步接收回调 */
    this->serial_driver_->port()->async_receive( std::bind( &ChassisNode::read_sensors_data, this, std::placeholders::_1, std::placeholders::_2));
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

    if( this->sfp_->get_command( &(this->ring_buffer_), command, &command_size) ) {
        /* 本项目传感器传数据帧的 数据字段长度 默认为 40, 效率为 40/43=93% */
        if( 40 == command_size ) {
            RCLCPP_INFO(this->get_logger(), "获取了一帧数据( %u bytes )", command_size);
            this->publish_sensors_data( command, command_size);
        } else {
            /* 未知数据 */
            RCLCPP_WARN(this->get_logger(), "(解析帧得到了 %u bytes 未知数据): %.*s", command_size, command_size, (const char*)command);
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "接收了不完整或者错误的帧");
    }
}

void ChassisNode::publish_sensors_data( uint8_t* data, uint16_t size)
{
    /* 由于传感器数据的长度固定是40, 所以这里的 size 变量直接告知编译器故意不使用( 防止产生Warning ) */
    (void)size;

    float linear_x_speed = 0, linear_y_speed  = 0, angular_z_speed = 0;    //  值得注意的是这里的 xy是线速度, z是角速度
    float imu_acce_x = 0, imu_acce_y = 0, imu_acce_z = 0;
    float imu_gyro_x = 0, imu_gyro_y = 0, imu_gyro_z = 0;
    float battery_capacity = 0;

    /* 临时定义一个反序列化宏函数用于处理数据, 值得注意的是Ubuntu一般是小端 */
    uint16_t index = 0, copied = 0;
    #define deserialization(var) copied = sizeof( var ); memcpy( &var, &( data[index] ), copied); index += copied;
    deserialization(linear_x_speed);    deserialization(linear_y_speed);    deserialization(angular_z_speed);
    deserialization(imu_acce_x);        deserialization(imu_acce_y);        deserialization(imu_acce_z);
    deserialization(imu_gyro_x);        deserialization(imu_gyro_y);        deserialization(imu_gyro_z);
    deserialization(battery_capacity);
    #undef deserialization

    /* 获取时间戳 */
    this->current_time_ = this->now();
    
    /* 记录位移与姿态数据 */
    float dt = ( this->current_time_ - this->last_time_ ).seconds();
    this->x_ += ( linear_x_speed * cos( this->yaw_ ) - linear_y_speed * sin( this->yaw_ ) ) * dt;
    this->y_ += ( linear_x_speed * sin( this->yaw_ ) + linear_y_speed * cos( this->yaw_ ) ) * dt;
    this->yaw_ += angular_z_speed * dt;

    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion odom_quat;
    /* 发布里程计数据(交由 robot localization 处理) */
    quat_tf.setRPY(0, 0, this->yaw_);
    odom_quat = tf2::toMsg(quat_tf); 

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = this->current_time_;
    odometry.header.frame_id = this->odom_frame_;
    odometry.pose.pose.position.x = this->x_;
    odometry.pose.pose.position.y = this->y_;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = odom_quat;
    
    odometry.child_frame_id = this->base_frame_;
    odometry.twist.twist.linear.x = linear_x_speed;
    odometry.twist.twist.linear.y = linear_y_speed;
    odometry.twist.twist.angular.z = angular_z_speed;


    if ( this->is_static( linear_x_speed, linear_y_speed, angular_z_speed) ) {
        std::copy( odom_pose_covariance_static.begin(), odom_pose_covariance_static.end(), odometry.pose.covariance.begin());
        std::copy( odom_twist_covariance_static.begin(), odom_twist_covariance_static.end(), odometry.twist.covariance.begin());
    } else {
        std::copy( odom_pose_covariance_dynamic.begin(), odom_pose_covariance_dynamic.end(), odometry.pose.covariance.begin());
        std::copy( odom_twist_covariance_dynamic.begin(), odom_twist_covariance_dynamic.end(), odometry.twist.covariance.begin());
    }
    
    this->odom_publish_->publish( odometry);

    /* 发布IMU数据(交由 robot localization 处理) */
    Imu_Quaternion quaternion;
    mahony_filter( &quaternion, imu_gyro_x, imu_gyro_y, imu_gyro_z, imu_acce_x, imu_acce_y, imu_acce_z);
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = this->current_time_;
    imu.header.frame_id = this->imu_frame_;
    imu.orientation.x = quaternion.x;
    imu.orientation.y = quaternion.y;
    imu.orientation.z = quaternion.z;
    imu.orientation.w = quaternion.w;
    imu.orientation_covariance[0] = 1e6;        // roll     方差很大,   表示不可信(因为本项目机器人工作在平面)
    imu.orientation_covariance[4] = 1e6;        // pitch    方差很大,   表示不可信(因为本项目机器人工作在平面)  
    imu.orientation_covariance[8] = 1e-6;       // yaw      方差很小,   表示可信

    imu.linear_acceleration.x = imu_acce_x; 
    imu.linear_acceleration.y = imu_acce_y;
    imu.linear_acceleration.z = imu_acce_z;
    // imu.linear_acceleration_covariance = ;   //  忽略

    imu.angular_velocity.x = imu_gyro_x;
    imu.angular_velocity.y = imu_gyro_y;
    imu.angular_velocity.z = imu_gyro_z;
    imu.angular_velocity_covariance[0] = 1e6;   // x轴角速度  方差很大,  表示不可信(因为本项目机器人工作在平面)
    imu.angular_velocity_covariance[4] = 1e6;   // y轴角速度  方差很大,  表示不可信(因为本项目机器人工作在平面)
    imu.angular_velocity_covariance[8] = 1e-6;  // z轴角速度  方差很小,  表示可信

    this->imu_publish_->publish( imu);

    /* 记录时间戳 */
    this->last_time_ = this->current_time_;

    // RCLCPP_INFO( this->get_logger(), "前进速度 %.2f, 转向速度 %.2f", linear_x_speed, angular_z_speed);
    // RCLCPP_INFO( this->get_logger(), "姿态  %.2f %.2f %.2f", linear_x_speed, angular_z_speed, angular_z_speed);
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

    uint8_t frame[64] = {0};
    uint8_t data[64] = {0};
    uint16_t index = 0, copied = 0;
    float linear_x_speed = msg.linear.x, linear_y_speed = msg.linear.y, angular_z_speed = msg.angular.z;    //  值得注意的是这里的 xy是线速度, z是角速度
    
    /* 临时定义一个序列化宏函数用于处理数据, 值得注意的是Ubuntu一般是小端 */
    #define serialization(var) copied = sizeof( var ); memcpy( &( data[index] ), &var, copied); index += copied;
    serialization( linear_x_speed); serialization( linear_y_speed); serialization( angular_z_speed); 
    #undef serialization

    /* 生成帧 */
    uint16_t frame_size = this->sfp_->create_frame( frame, data, index);
    
    /* 串口发送数据 */
    try {
        std::vector<uint8_t> transmit_data_buffer( frame, frame+frame_size);
        auto port = this->serial_driver_->port();
        port->send( transmit_data_buffer );
    } catch(const std::exception &ex) {
        RCLCPP_ERROR( this->get_logger(), "串口传输发生错误 %s",ex.what());
    }
}

/**
  * @brief 判断底盘是否处于静止状态
  *
  * @param linear_x  x方向的线速度
  * @param linear_y  y方向的线速度
  * @param angular_z z方向的角速度
  * @return
  *      - is_static: 处于静止为 true, 否则为 false
  */
bool ChassisNode::is_static( float linear_x, float linear_y, float angular_z)
{
    float linear_x_abs  = std::abs(linear_x);
    float linear_y_abs  = std::abs(linear_y);
    float angular_z_abs = std::abs(angular_z); 

    if( ( linear_x_abs < 0.001 ) && ( linear_y_abs < 0.001 ) && ( angular_z_abs < 0.01 ) )
        return true;
    else
        return false;
}