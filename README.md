# YuanGod 智能车

#  项目来源
处于大学学习，碰巧研究道教中，以**道教概念**为这个项目命名，起名为**元神**智能车(本人英语不行，故用中式英语其名为yuangod)

# TCP与虚拟串口双向通讯
测试命令
``` bash
# 监听对应端口，可以读取虚拟串口发来的数据或者向虚拟串口发送数据
telnet localhost 端口号
# 用虚拟串口向TCP端口发送数据 ttyVIRT0 可改为对应虚拟串口
sudo sh -c 'echo "hello world" > /dev/ttyVIRT0'
# 读取TCP端口发往虚拟串口的数据 ttyVIRT0 可改为对应虚拟串口
sudo cat /dev/ttyVIRT0
# 关闭socat进程
sudo pkill socat 或者 sudo killall socat
# 查询socat进程
ps aux | grep socat
```
