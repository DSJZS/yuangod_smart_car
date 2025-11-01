#!/bin/bash #脚本解释器为shell

# 创建虚拟串口并监听TCP
# 如果想要检查是否创建成功，测试方式可以参考README.md相关说明
# 1.底盘
sudo sh -c 'socat -d -d PTY,link=/dev/ttyVIRT0,b115200,raw,echo=0,group=dialout,mode=660 TCP-LISTEN:8899,fork,reuseaddr &'
# 2.雷达
sudo sh -c 'socat -d -d PTY,link=/dev/ttyVIRT1,b115200,raw,echo=0,group=dialout,mode=660 TCP-LISTEN:8888,fork,reuseaddr &'
