/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-15 17:25:00
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-18 14:54:42
 */
#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <sys/times.h>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <sstream>
#include "imu/IMUData.h"

#define IMU_PORT    "/dev/ttyUSB5"
#define PUB_TOPIC   "imu_get"

#define IMU_DATA_LEN    59

imu::IMUData imuData;
serial::Serial mySerial; //声明串口对象

union float_to_char{
    char ch[4];
    float fl;
    int in;
    unsigned int uin;
};

char Imu_check_data(char* buf, char cnt){
    int i;
    char data = 0;
    for (i = 0; i < cnt; i++)	{
        data += buf[i];
    }
    //	printf("buf=%-3x%-3x%-3x%-3x\r\n",buf[0],buf[1],buf[2],data&0xff);
    return data & 0xff;
}

void ImuAnalysisDataArray(char* data){
    union float_to_char chtofl;
    int cnt = 2, idx = 0;
    for (idx = 0; idx < 4; idx++)	{
        chtofl.ch[idx] = data[cnt++];
    }
    imuData.X_GYRO_OUT = chtofl.fl;
    for (idx = 0; idx < 4; idx++)	{
        chtofl.ch[idx] = data[cnt++];
    }
    imuData.Y_GYRO_OUT = chtofl.fl;
    for (idx = 0; idx < 4; idx++)	{
        chtofl.ch[idx] = data[cnt++];
    }
    imuData.Z_GYRO_OUT = chtofl.fl * 1.001f;
    for (idx = 0; idx < 4; idx++)	{
        chtofl.ch[idx] = data[cnt++];
    }
    imuData.X_ACCL_OUT = chtofl.fl * 9.80665f;
    for (idx = 0; idx < 4; idx++)	{
        chtofl.ch[idx] = data[cnt++];
    }
    imuData.Y_ACCL_OUT = chtofl.fl * 9.80665f;
    for (idx = 0; idx < 4; idx++)	{
        chtofl.ch[idx] = data[cnt++];
    }
    imuData.Z_ACCL_OUT = chtofl.fl * 9.80665f;

    /*逆时针旋转90°*/
    float imu_temp = 0;

    imu_temp = imuData.X_GYRO_OUT;
    imuData.X_GYRO_OUT = -imuData.Y_GYRO_OUT;
    imuData.Y_GYRO_OUT = imu_temp;

    imu_temp = imuData.X_ACCL_OUT;
    imuData.X_ACCL_OUT = -imuData.Y_ACCL_OUT;
    imuData.Y_ACCL_OUT = imu_temp;

    if (isnormal(imuData.X_ACCL_OUT))
        return;
    if (isnormal(imuData.Y_ACCL_OUT))
        return;
    if (isnormal(imuData.Z_ACCL_OUT))
        return;
    if (isnormal(imuData.X_GYRO_OUT))
        return;
    if (isnormal(imuData.Y_GYRO_OUT))
        return;
    if (isnormal(imuData.Z_GYRO_OUT))
        return;
    imuData.lastTime = imuData.time;
    
    struct timeval tv;
    gettimeofday(&tv,NULL);
    imuData.time = tv.tv_sec;

    imuData.getNewDataFlag = 1;
}

void parse_imu102n_data(char* rx){
    char* myrx = rx;
    static int UsartImu102nRxErr[10] = {0};

    /* 帧头校验 */
    if (myrx[0] != 0x5a || myrx[1] != 0x5a)	{
        UsartImu102nRxErr[0]++;
        return;
    }

    /* 帧长校验 */
    //	if(rx1_decode_databuf[3]+5>len)
    //	{
    //		UsartImu102nRxErr[1]++;
    //		return;
    //	}

    /* 和校验 */
    if (myrx[58] == Imu_check_data(&myrx[2], 56))	{
    }
    else	{
        UsartImu102nRxErr[2]++;
        return;
    }

    /* 数据提取 */
    ImuAnalysisDataArray(myrx);
}
//230400
int main(int argc, char** argv){
    printf("This is IMU Get node\n");
    ros::init(argc, argv, "imu_node");       //a)
    ros::NodeHandle n;                           //b)
    ros::Publisher read_pub = n.advertise<imu::IMUData>(PUB_TOPIC, 1000);

    try{
        //串口设置
        mySerial.setPort(IMU_PORT);
        mySerial.setBaudrate(230400);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        mySerial.setTimeout(to);
        mySerial.open();
        mySerial.flush();
    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to Open Serial Port !");
        return -1;
    }

    if (mySerial.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else{
        return -1;
    }

    ros::Rate loop_rate(20);
    while (ros::ok()){
        if (mySerial.available()){
            std::string recv;
            imu::IMUData tempIMUData;
            int len = mySerial.available();
            if (len >= IMU_DATA_LEN){
                recv = mySerial.read(2);
		if (recv[0] == 0x5A && recv[1] == 0x5A){
			std::string tempRecv = mySerial.read(IMU_DATA_LEN - 2);
			recv = recv + tempRecv;
                	parse_imu102n_data((char*)recv.c_str());
			printf("\n========IMU========\nXacc:%f\tYacc:%f\tZacc:%f\nXgyro:%f\tYgyro:%f\tZgyro:%f\nTime:%d\n", imuData.X_ACCL_OUT, imuData.Y_ACCL_OUT, imuData.Z_ACCL_OUT, imuData.X_GYRO_OUT, imuData.Y_GYRO_OUT, imuData.Z_GYRO_OUT, imuData.time);
		}
            }
        }

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
