/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-02 09:11:07
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-02 15:40:03
 */
#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

serial::Serial sendSerial; //声明串口对象

#define PORT        "/dev/ttyUSB0"
#define ROBOT_ID    0x00

#define CMD_A1SIZE              32
#define CMD_MOVECTRL            0xA1

unsigned char Checksum(unsigned char* buf, unsigned int length){
    unsigned char index;
    unsigned short int tempResult = 0x0000;
    unsigned char checksum = 0;
    for (index = 0; index < length; index++)	{
        tempResult += buf[index];
    }
    checksum = (tempResult & 0xff);
    return checksum;
}

void A1DataPack(unsigned char* data, int move){
    data[0] = 0x55;
    data[1] = 0xAA;
    data[2] = ROBOT_ID;

    data[3] = CMD_A1SIZE - 5;
    data[4] = CMD_MOVECTRL;

    data[5] = move;

    data[CMD_A1SIZE - 1] = Checksum(&data[4], data[3]);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Get Move: [%s]", msg->data.c_str());
    int move = 0;
    unsigned char data[CMD_A1SIZE] = { 0 };

    sscanf(msg->data.c_str(), "move:%d", &move); // Get move
    A1DataPack(data, move);
    sendSerial.write(data, CMD_A1SIZE);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "send");       //a)
    ros::NodeHandle n;                           //b)
    ros::Subscriber sub = n.subscribe("message", 1000, chatterCallback);
    char *port = argv[1];
    printf("port=[%s]\n", port);
    
    try{
        //串口设置
        sendSerial.setPort(port);
        sendSerial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        sendSerial.setTimeout(to);
        sendSerial.open();
    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to Open Serial Port !");
        return -1;
    }
    ROS_INFO("Open [%s] success", port);
    ROS_INFO("Waiting command...");
    ros::spin();                 //d)

    return 0;
}