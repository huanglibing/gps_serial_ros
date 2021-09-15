/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-02 09:11:07
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-15 16:37:16
 */
#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "control/CarData.h"
#include "../include/datapack.h"

serial::Serial mySerial; //声明串口对象

#define PKG_LEN     24
#define PKG_HEAD    0x7B
#define PKG_TAIL    0x7D

#define PORT        "/dev/ttyUSB1"
#define TOPIC       "send car"
#define PUB_TOPIC   "read car"

void chatterCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Get CMD: [%s]", msg->data.c_str());
    mySerial.write((const unsigned char*)msg->data.c_str(), CMD_A1SIZE);
}

static int GetCRC(const char* data, int dataLen){
    int i, CRC = 0;
    for (i = 0; i < dataLen; i++){
        CRC = CRC^data[i];
    }
    return CRC;
}

static control::CarData ParseCarData(std::string recvData){
    control::CarData data;

    data.flag_stop = recvData[1];

    data.Xspeed = recvData[2] << 8 | recvData[3];
    data.Yspeed = recvData[4] << 8 | recvData[5];
    data.Zspeed = recvData[6] << 8 | recvData[7];

    data.Xacc = recvData[8]  << 8 | recvData[9];
    data.Yacc = recvData[10] << 8 | recvData[11];
    data.Zacc = recvData[12] << 8 | recvData[13];

    data.Xangv = recvData[14] << 8 | recvData[15];
    data.Yangv = recvData[16] << 8 | recvData[17];
    data.Zangv = recvData[18] << 8 | recvData[19];

    data.batteryVol = recvData[20] << 8 | recvData[21];

    return data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "control_node");       //a)
    ros::NodeHandle n;                           //b)
    ros::Subscriber sub = n.subscribe(TOPIC, 1000, chatterCallback);
    ros::Publisher read_pub = n.advertise<control::CarData>(PUB_TOPIC, 1000);
    
    try{
        //串口设置
        mySerial.setPort(PORT);
        mySerial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        mySerial.setTimeout(to);
        mySerial.open();
    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to Open Serial Port !");
        return -1;
    }

    if (mySerial.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else          {
        return -1;
    }
    //指定循环的频率 
    ros::Rate loop_rate(50);
    while (ros::ok()){
        if (mySerial.available()){
            ROS_INFO_STREAM("Reading from serial port\n");

            std::string recv;
            control::CarData carData;
            int len = mySerial.available();
            if (len >= PKG_LEN){
                recv = mySerial.read(PKG_LEN);
                if (recv[0] == PKG_HEAD && recv[PKG_LEN - 1] == PKG_TAIL){
                    if (recv[PKG_LEN - 1] == GetCRC(recv.c_str(), PKG_LEN - 1)){
                        ROS_INFO_STREAM("Read: " << recv);
                        carData = ParseCarData(recv);
                        read_pub.publish(carData);
                    }
                }
            }
        }

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}