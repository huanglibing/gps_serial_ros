/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-15 17:25:00
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-15 17:33:12
 */
#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "imu/IMUData.h"
imu::IMUData imuData;

int main(void){
    printf("This is IMU Get node\n");
    return 0;
}