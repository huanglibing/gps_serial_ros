/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-02 09:11:07
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-18 09:19:45
 */
#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "car/CarData.h"

serial::Serial mySerial; //声明串口对象

#define PKG_LEN     24
#define PKG_HEAD    0x7B
#define PKG_TAIL    0x7D

#define PORT        "/dev/ttyUSB4"
#define TOPIC       "send_to_car"
#define PUB_TOPIC   "read_from_car"

void chatterCallback(const std_msgs::String::ConstPtr& msg){
        ROS_INFO("[%s]", msg->data.c_str());
    
    // mySerial.write((const unsigned char*)msg->data.c_str(), CMD_A1SIZE);
}

static int GetCRC(const char* data, int dataLen){
    int i, CRC = 0;
    for (i = 0; i < dataLen; i++){
        CRC = CRC^data[i];
    }
    return CRC;
}

float GetV(short data){
    if (data < 0){
    	return (float)(data*(-1)) / 1000;
    }
    else{
    	return (float)data / 1000;
    }	
}

float GetAcc(short data){
    if (data < 0){
    	return (float)(data*(-1)) / 1672;
    }
    else{
    	return (float)data / 1672;
    }	
}

float GetAngV(short data){
    if (data < 0){
    	return (float)(data*(-1)) / 3753;
    }
    else{
    	return (float)data / 3753;
    }
}

static car::CarData ParseCarData(std::string recvData){
    car::CarData data;
    short tempdata = 0;

    data.flag_stop = recvData[1];

    tempdata = (recvData[2] << 8 | recvData[3]);
    data.Xspeed = GetV(tempdata);
    tempdata = (recvData[4] << 8 | recvData[5]);
    data.Yspeed = GetV(tempdata);
    tempdata = (recvData[6] << 8 | recvData[7]);
    data.Zspeed = GetV(tempdata);

    tempdata = recvData[8] << 8 | recvData[9];
    data.Xacc = GetAcc(tempdata);
    tempdata = (recvData[10] << 8 | recvData[11]);
    data.Yacc = GetAcc(tempdata);
    tempdata = (recvData[12] << 8 | recvData[13]);
    data.Zacc = GetAcc(tempdata);

    tempdata = (recvData[14] << 8 | recvData[15]);
    data.Xangv = GetAngV(tempdata);
    tempdata = (recvData[16] << 8 | recvData[17]);
    data.Yangv = GetAngV(tempdata);
    tempdata = (recvData[18] << 8 | recvData[19]);
    data.Zangv = GetAngV(tempdata);

    data.batteryVol = (float)(recvData[20] << 8 | recvData[21]) / 1000;

    return data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "car_node");       //a)
    ros::NodeHandle n;                           //b)
    ros::Subscriber sub = n.subscribe(TOPIC, 1000, chatterCallback);
    ros::Publisher read_pub = n.advertise<car::CarData>(PUB_TOPIC, 1000);
    
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
    ros::Rate loop_rate(20);
    while (ros::ok()){
        if (mySerial.available()){
            std::string recv;
            car::CarData carData;
            int len = mySerial.available();
            if (len >= PKG_LEN){
                recv = mySerial.read(PKG_LEN);
		/*for (int k = 0; k<PKG_LEN; k++){
			printf("[%d]:0x%X\n", k, recv.c_str()[k]);
		}*/
                if (recv[0] == PKG_HEAD && recv[PKG_LEN - 1] == PKG_TAIL){
                    if (recv[PKG_LEN - 2] == GetCRC(recv.c_str(), PKG_LEN - 2)){
                        carData = ParseCarData(recv);
			printf("\n==========CarData===========\nXv:%f\tYv:%f\tZv:%f\tm/s\nXac:%f\tYac:%f\tZac:%f\tm/s2\nXangv:%f\tYangv:%f\tZangv:%f\trad/s\nbatteryVol:%f\tV\n", carData.Xspeed, carData.Yspeed, carData.Zspeed, carData.Xacc, carData.Yacc, carData.Zacc, carData.Xangv, carData.Yangv, carData.Zangv, carData.batteryVol);
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
