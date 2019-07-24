#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <std_msgs/Empty.h> 
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>//string转化为double
#include <iomanip>//保留有效小数
#include "serial_Port/GPS.h"
serial::Serial ser; //声明串口对象

//解析GPS
void RecePro(std::string s , double& lat , double& lon )
{
    //分割有效数据，存入vector中
    std::vector<std::string> v;
    std::string::size_type pos1, pos2;
    pos2 = s.find(",");
    pos1 = 0;
    while ( std::string::npos !=pos2 )
    {
        v.push_back( s.substr( pos1, pos2-pos1 ) );
        pos1 = pos2 + 1;
        pos2 = s.find(",",pos1);
    }
    if ( pos1 != s.length() )
        v.push_back( s.substr( pos1 ));
    //解析出经纬度
    if (v.max_size() >= 6 && (v[6] == "1" || v[6] == "2" || v[6] == "3" || v[6] == "4" || v[6] == "5" || v[6] == "6" || v[6] == "8" || v[6] == "9"))
    {
        //纬度
        if (v[2] != "") lat = std::atof(v[2].c_str()) / 100;
        int ilat = (int)floor(lat) % 100;
        lat = ilat + (lat - ilat) * 100 / 60;
        //经度
        if (v[4] != "") lon = std::atof(v[4].c_str()) / 100;
        int ilon = (int)floor(lon) % 1000;
        lon = ilon + (lon - ilon) * 100 / 60;

    }
}
int main(int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_node");
    //声明节点句柄
    ros::NodeHandle nh;
    //注册Publisher到话题GPS
    ros::Publisher GPS_pub = nh.advertise<serial_Port::GPS>("GPS",1000);
    try
    {
      //串口设置
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(9600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open Serial Port !");
        return -1;
    }
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    //设置循环的频率 20HZ
    ros::Rate loop_rate(50);

    std::string strRece;
    while (ros::ok())
    {

        if (ser.available())
        {
            //1.读取串口信息：
            //ROS_INFO_STREAM("Reading from serial port\n");

            //通过ROS串口对象读取串口信息
            strRece += ser.read(ser.available());
            std::cout << strRece ;


        }
    ros::spinOnce();
    loop_rate.sleep();
    }
}

