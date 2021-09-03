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
#include <iostream>
#include "serial_Port/GPS.h"
serial::Serial ser; //声明串口对象
serial_Port::GPS GPS_data;//全局变量，解析后数据
int debug_break[10];
float debug_break_float[10];


const unsigned char  HEX_TO_ASCII[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

/**********************************************************************************************
函数名称: unsigned char  CheckGpsData( unsigned char * Addr )
功    能: 校验GPS数据
输    入: null
输    出: null
日    期：7.8
作    者： 
**********************************************************************************************/
unsigned char  CheckGpsData( std::string s )
{
    unsigned char  crc8 = 0 ,i = 0 ;

    while( s[i] != '$' )
    {
        if( ( s[i] == '\0' ) || ( s[i] == '*' ) ) return 0;

        i++;
    }

    i++;
    crc8 = s[i++];

    while( s[i] != '*' )
    {
        if( s[i] == '\0' ) return 0;

        crc8 ^= s[i++];
    }

    i++;

    if( HEX_TO_ASCII[( crc8 >> 4 ) & 0x0f] == s[i++] )
    {
        if( HEX_TO_ASCII[crc8 & 0x0f] == s[i] )
        {
            return 1;
        }
    }

    return 0;
}



/**********************************************************************************************
函数名称: RecePro(std::string s,int len)
功    能: 解析接收到的数据，//提取GGA,RMC中数据
输    入: 数据缓存指针，数据长度
输    出: null
日    期：8.2
作    者：
**********************************************************************************************/
int RecePro(std::string s,int len)
{
  int start = -1, commanum = 0, BufIndex = 0 ,i = 0 ,end_out=-1;  //临时变量     
  std::string str;		//临时变量，一帧数据
  std::string   gpsNum;	 	//星数
  std::string   gpsStatue ;	//定位状态
  std::string   gpsHdop;    	//海拔高度 
  std::string   LatitudeBuf;	//纬度
  std::string   LongitudeBuf;	//经度 
  std::string   AltitudeBuf;    	//海拔高度 
  std::string   temp_str; 	//临时变量


  /*******************************************************GGA语句解析部分**************************************************************/
  start = s.find("GGA"); //从缓存里找到GGA开头数据，字符串所在地址
  if(start>0)	//找到字符串开头
  {
    str=s.substr(start-3);
    if(CheckGpsData(str))	//CRC校验，校验成功
    {
      //变量初始化
      commanum = 0;
      i = 0;
      temp_str.clear();

      /************************解析GGA数据，提取需要的参数*******************/
      do
      {
        if( ( str[i++] ) == ',' )
        {
          commanum++;
          BufIndex = 0;
        }

        if( str[i] != ',' ) 
        {
          switch( commanum )
          {
            case 2://纬度
            if( BufIndex < ( 17) )
            {
              temp_str = str.substr(i,1);
              LatitudeBuf+=temp_str;
            }				    
            break;
            case 4://经度
            if( BufIndex < ( 17) )
            {
              temp_str = str.substr(i,1);
              LongitudeBuf+=temp_str;
            }				    
            break;
            case 6://状态
            if( BufIndex < ( 2) )
            {
              temp_str = str.substr(i,1);
              gpsStatue+=temp_str;
            }
            break;
            case 7://卫星数 
            if( BufIndex < ( 2) )
            {
              temp_str = str.substr(i,1);
              gpsNum+=temp_str;
            }
            break;
            case 8://水平精度因子
            if( BufIndex < ( 5) )
            {
              temp_str = str.substr(i,1);
              gpsHdop+=temp_str;
            }
            break;

            default:
            break;
          }

          if( commanum > 14 )
          break;

          BufIndex++;
          }
        }
        while( str[i] != '*' );
	
	//已经解析数据位置	
        if(end_out<(start+i+2))
        {end_out=start+i+2;}

        //调试用，后期待删除
        std::cout << "endout:" << end_out << " start:" << start << " i:" << i << " len:" << s.length() << " endstr:" << s.substr(end_out-5) ;
        std::cout << "lat:" << LatitudeBuf<< " lon:" << LongitudeBuf << " S:" << gpsStatue << " num:" << gpsNum << " hdop:" << gpsHdop << "\r\n";
        //std::cout << str ;
        /****************************解析GGA数据完成******************************/
        if(commanum==14)
        {
          GPS_data.lat = atof(LatitudeBuf.c_str())/100;
          GPS_data.lon = atof(LongitudeBuf.c_str())/100;
          GPS_data.gpsStatue = atoi(gpsStatue.c_str());
          GPS_data.gpsNum = atoi(gpsNum.c_str());
          GPS_data.gpsHdop = atof(gpsHdop.c_str());
        }
    }//校验
  }//帧头

  return end_out;

}

//主函数
int main(int argc, char** argv)
{
  static int len;
  static int len_total;

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

  //设置循环的频率 50HZ 20ms 要求循环频率大于数据接收频率
  ros::Rate loop_rate(50);

  std::string strRece;
  while (ros::ok())
  {
    //获取数据长度
    len = ser.available();     

    if (len>0)    //接收数据
    {  
      //通过ROS串口对象读取串口信息，存放于缓冲区
      strRece += ser.read(ser.available());
      //std::cout << strRece ;  

      //模拟数据
      //strRece = "$GNGGA,075026.000,2231.9112,N,11356.1548,E,1,13,1.2,1.4,M,0.0,M,,*71\r\n";      
    }
  
    //数据处理，提取有效数据
    int Rece_out=RecePro(strRece,len_total);    

    //缓冲区处理，清除已经提取的数据
    if(Rece_out>0)
    {strRece = strRece.substr(Rece_out);debug_break[0]++;}

    //防止缓冲区过大，3s数据量 2100=700×3
    if(strRece.length()>2100)
    {strRece.clear();debug_break[1]++;}

    //发布话题消息
    if(Rece_out>0)
    {GPS_pub.publish(GPS_data);debug_break[2]++;}

    //断点数据分析，后期待删除
    static int debug_100ms=0;
    debug_100ms++;
    if(debug_100ms >= 5) //5*20ms=100ms
    {
      std::cout << "Rece_out:" << Rece_out<< " len:" << strRece.length() << " b1:" << debug_break[0]<< " b2:" << debug_break[1]<< " b3:" << debug_break[2]  << "\r\n";	
      //std::cout << "f1:" << debug_break_float[0]<< " f2:" << debug_break_float[1] << " f3:" << debug_break_float[2]  << "\r\n";
      debug_100ms=0;    
    }

    
    ros::spinOnce();
    loop_rate.sleep();
  }
}


