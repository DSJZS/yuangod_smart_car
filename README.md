# YuanGod 智能车

#  项目来源
处于大学学习，碰巧研究道教中，以**道教概念**为这个项目命名，起名为**元神**智能车(本人英语不行，故用中式英语其名为yuangod)

# 项目组成
1. 底盘: 由ESP32-S3控制，负责控制车轮并读取IMU、车轮速度、电池容量等数据通过TCP转发给服务器
2. 雷达: 由ESP32-S3控制，负责读取雷达的数据并通过UDP转发给ROS2服务器

# 一些测试用的命令
## TCP与虚拟串口双向通讯测试
``` bash
# 监听对应端口，可以读取虚拟串口发来的数据或者向虚拟串口发送数据
telnet localhost 端口号

# 用虚拟串口向TCP端口发送数据 ttyVIRT0 可改为对应虚拟串口
sudo sh -c 'echo "hello world" > /dev/ttyVIRT0'

# 读取TCP端口发往虚拟串口的数据 ttyVIRT0 可改为对应虚拟串口
# 值得注意的是如果终端从串口缓存中读取了数据， 那么ROS2节点就无法读取到
sudo cat /dev/ttyVIRT0

# 关闭socat进程
sudo pkill socat 或者 sudo killall socat

# 查询socat进程
ps aux | grep socat
```

