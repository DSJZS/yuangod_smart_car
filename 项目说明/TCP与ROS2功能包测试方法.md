# 测试方法介绍
> 介于本人的水平，难免有所简陋，这里提供一些测试TCP连接或者ROS2功能包通讯是否正常的方法( 平台为 Ubuntu )

# 测试TCP连接
## 用 telnet 测试
### 命令
``` bash
telnet localhost 端口号
```
### 使用方法介绍
对应端口接收的数据会在终端之中所有显示
想要通过端口发送则可以直接在端口输入数据并按下 `回车(Enter)` 进行发送
退出方法为 `Ctrl + ]` 之后按下 `q`，最后再按下 `回车(Enter)`

## socat
### 命令
``` bash
socat - TCP:localhost:端口号
```
### 使用方法介绍
与 telnet 使用方法一致, 终端接收和发送
关闭方法为 `Ctrl + c`

# 测试ROS2功能包通讯
## 个人习惯方法
``` bash
mkfifo test_fifo
socat ./test_fifo TCP:localhost:端口号
cat test_msg.bin > test_fifo
rm test_fifo
```
使用 mkfifo 创建一个命名管道并与对应的端口号进行连接
这样可以直接将二进制测试文件内容传输到对应端口
可以通过日志或者rqt等工具查看ROS2功能包创建的节点是否正常接收并处理文件