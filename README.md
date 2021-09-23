# gps_serial_ros 必读

此包内含car节点，controlcar节点，gpspub节点，imu节点

- gpspub用于发布GPS数据，端口默认ttyTHS0

    gpspub发布话题为"GPS", GPS数据结构见`gpspub/msg/GPS.msg`

- imu节点用于获取IMU传感器数据，默认ttyUSB5

- car节点用于获取小车数据和转发小车控制命令，默认ttyUSB4

- controlcar节点用于发送小车控制命令到car节点，可使用键盘方向键控制小车运动。

