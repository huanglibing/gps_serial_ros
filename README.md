# gps_serial_ros 必读

此包内含control节点和gpspub节点

- gpspub用于发布GPS数据，端口默认ttyUSB0

    gpspub发布话题为"GPS", GPS数据结构见`gpspub/msg/GPS.msg`

- control用于接收其他节点的控制指令，解析后发往小车, 端口默认ttyUSB1

    control订阅话题为"message", 格式为`"move:%d"`, %d为1时前进，0停止

参考资料：ROS实现串口GPS数据的解析与通信 https://blog.csdn.net/weixin_43795921/article/details/85219249
## 第一步：安装serial包实现串口通信
参考链接：https://blog.csdn.net/liuxiaodong400/article/details/90370927

### 如果打不开串口，请修改权限
参考链接：https://www.codenong.com/cs106083177/

### 如果系统为Ubuntu 20.04
在`/src/serial/build` 目录下cmake .. && make && sudo make install，即可使用serial串口

参考: https://blog.csdn.net/Shushan1/article/details/118976650

## 第二步：编译源码

ROS工作区目录运行: catkin_make

## 第三步：运行


    $ source ./devle/setup.bash 
    $ roslaunch gps_control_launch gps_control.launch




# 自测试

#### 1.ROS工作区根目录运行自定义的**GPS订阅节点**，订阅话题为"GPS"：

gpspub内有例程，如下运行：

        $ source ./devel/setup.bash
        $ rosrun gpspub listener
        
listener会打印来自"GPS"话题的消息


GPS消息结构详见 `src/gpspub/msg/GPS.msg`
    
### 2. 控制小车

#### 1.运行发布小车控制命令节点:

control节点内有例程，如下运行：
    
        $ source ./devel/setup.bash
        $ rosrun control testpub
        
testpub自动向message话题发布消息，可观察到小车轮子循环启动与停止。
