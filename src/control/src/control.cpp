/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-02 09:11:07
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-03 11:21:19
 */
#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "datapack.h"

serial::Serial sendSerial; //声明串口对象

#define PORT        "/dev/ttyUSB1"
#define TOPIC       "message"

void chatterCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Get Move: [%s]", msg->data.c_str());
    int move = 0;
    unsigned char data[CMD_A1SIZE] = { 0 };

    sscanf(msg->data.c_str(), "move:%d", &move); // Get move
    A1DataPack(data, move);
    // for (int i=0; i < CMD_A1SIZE; i++){
    //     printf("[%d]=%d\n", i, data[i]);
    // }
    sendSerial.write((const uint8_t *)data, CMD_A1SIZE);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "send");       //a)
    ros::NodeHandle n;                           //b)
    ros::Subscriber sub = n.subscribe(TOPIC, 1000, chatterCallback);
    char *port = argv[1];
    printf("port=[%s]\n", port);
    
    try{
        //串口设置
        sendSerial.setPort(PORT);
        sendSerial.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        sendSerial.setTimeout(to);
        sendSerial.open();
    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to Open Serial Port !");
        return -1;
    }
    ROS_INFO("Open [%s] success", PORT);
    ROS_INFO("Waiting command...");
    ros::spin();                 //d)

    return 0;
}