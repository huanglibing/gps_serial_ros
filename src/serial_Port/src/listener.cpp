#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial_Port/GPS.h"
#include <iomanip>
void chatterCallback(const serial_Port::GPSConstPtr& msg)
{
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(7) << "纬度：" << msg->lat << " 经度：" << msg->lon << " 定位状态：" << msg->gpsStatue << " 卫星：" << msg->gpsNum << " 水平精度：" << msg->gpsHdop << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("GPS", 1000, chatterCallback);
    ros::spin();
    return 0;
}

