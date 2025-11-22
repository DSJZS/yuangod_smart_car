#!/bin/bash

# 关闭所有 socat 进程
sudo pkill -f "socat -d -d PTY,link=/dev/ttyVIRT0,b115200,raw,echo=0,group=dialout,mode=660 TCP-LISTEN:8899,fork,reuseaddr"
sudo pkill -f "socat -d -d PTY,link=/dev/ttyVIRT1,b115200,raw,echo=0,group=dialout,mode=660 TCP-LISTEN:8888,fork,reuseaddr"