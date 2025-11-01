# 工程介绍
**ros2_jazzy**工程用于管理各个ROS2功能包  
> 值得注意的是取名 ros2_jazzy 是为指明了 ROS2 的版本为 Jazzy ，ROS2的相关依赖需要自行下载  

# 使用方法
在终端之中输入  
``` bash
. yuangod_start.sh
```

# 功能包介绍
## 用户功能包
### chassis_driver
#### 功能
用于与底盘上的ESP32进行TCP连接以接收各个传感器的信息并且发送到对应的topic,订阅对应的导航速度topic并将速度指令发送到底盘上的ESP32使其控制电机  

### yuangod_description
#### 功能
用于启动各种可视化功能包以及对应的节点(比如rviz2、robot_state_publisher、joint_state_publisher)  

## 第三方功能包

