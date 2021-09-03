# gps_serial_ros
参考资料：ROS实现串口GPS数据的解析与通信 https://blog.csdn.net/weixin_43795921/article/details/85219249
## 第一步：安装serial包实现串口通信
参考链接：https://blog.csdn.net/liuxiaodong400/article/details/90370927

### 如果系统为Ubuntu 20.04
在`/src/serial/build` 目录下cmake .. && make && make install，即可使用serial串口

参考: https://blog.csdn.net/Shushan1/article/details/118976650

## 第二步：编译源码

ROS工作区目录运行: catkin_make

## 第三步：运行

### 1.获取GPS

#### 1.ROS工作区根目录运行**GPS发布节点**，发布话题为"GPS"：

        $ source ./devel/setup.bash
        $ rosrun serial_Port serialPort GPS模块端口 (例：$ rosrun serial_Port serialPort /dev/ttyUSB0)
        
#### 2.ROS工作区根目录运行自定义的**GPS订阅节点**，订阅话题为"GPS"：

serial_Port内有例程，如下运行：

        $ source ./devel/setup.bash
        $ rosrun serial_Port listener
        
listener会打印来自"GPS"话题的消息


GPS消息结构详见 `src/serial_Port/msg/GPS.msg`
    
### 2. 控制小车

#### 1.运行小车控制节点：

        $ source ./devel/setup.bash
        $ rosrun control control 与小车的通信端口 (例：$ rosrun control control /dev/ttyUSB0)

小车订阅来自话题"message"的消息
        
#### 2.运行发布小车控制命令节点:

control节点内有例程，如下运行：
    
        $ source ./devel/setup.bash
        $ rosrun control testpub
        
testpub自动向message话题发布消息，可观察到小车轮子循环启动与停止。
