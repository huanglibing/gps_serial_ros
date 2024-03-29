#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>
#include <std_msgs/Empty.h> 
#include <sys/epoll.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdlib>//string转化为double
#include <iomanip>//保留有效小数
#include <termios.h>
#include <iostream>
#include "../include/UartCom.h"
#include "gpspub/GPS.h"
serial::Serial ser; //声明串口对象
gpspub::GPS GPS_data;//全局变量，解析后数据
ros::Publisher GPS_pub;
int debug_break[10];
float debug_break_float[10];

#define GPS_PORT  "/dev/ttyTHS0"

struct my_serial_obj{
  int      MsgCenterSocket; //socket套接字
  int      epfd;

  struct epoll_event ev, events[20];
}serial_obj;


const unsigned char  HEX_TO_ASCII[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void sig_handler(int signo)
{
  if (signo == SIGINT){
    printf("received SIGINT, exit...\n");
    exit(0);
  }
}

float _wrap_180(float bearing){
  /* value is inf or NaN */
  if (!isfinite(bearing)){
    return bearing;
  }

  int c = 0;

  while (bearing >= 180){
    bearing -= 360;

    if (c++ > 3){
      return 0;	//65535
    }
  }

  c = 0;

  while (bearing < -180){
    bearing += 360;

    if (c++ > 3){
      return 0;	//65535
    }
  }

  return bearing;
}

int CRC32Value(int i){
  int j;
  int ulCRC;
  ulCRC = i;

  for (j = 8;j > 0;j--)	{
    if (ulCRC & 1)		{
      ulCRC = 0xEDB88320 ^ (ulCRC >> 1);
    }
    else ulCRC >>= 1;
  }
  return ulCRC;
}


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
函数名称: char CheckGpsData_32b(char* Addr)
功    能 : 校验GPS数据
输    入 : null
输    出 : null
日    期：7.8
作    者：
* *********************************************************************************************/
int CheckGpsData_32b(char * Addr){
  int crc32 = 0, crc32_resulr = 0, temp1 = 0, temp2 = 0, j = 0;
  char crc_32[8] = { 0 };

  while (*Addr != '*')	{
    if (*Addr == '\0')  return 0;
    temp1 = (crc32 >> 8) & 0x00ffffff;
    temp2 = CRC32Value(((int)crc32 ^ *Addr) & 0xff);
    crc32 = temp1 ^ temp2;
    Addr++;
  }

  Addr++;

  for (j = 0;j < 8;j++)	{
    crc32_resulr = crc32_resulr << 4;
    if (*(Addr + j) >= 0x30 && *(Addr + j) <= 0x39)
      crc_32[j] = *(Addr + j) - 0x30;
    else crc_32[j] = *(Addr + j) - 0x57;

    crc32_resulr |= crc_32[j];
  }

  if (crc32 == crc32_resulr)	{
    return 1;
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
int RecePro(char* recvData)
{
  const char* str;
  gpspub::GPS temp_diff_data;

  short commanum = 0, BufIndex = 0;

  char UtcTimeBuf[11] = { 0 };    //时间
  char UtcDataBuf[8] = { 0 };    //日期
  char LatitudeBuf[17] = { 0 };    //纬度
  char LongitudeBuf[17] = { 0 };    //经度
  char AltitudeBuf[8] = { 0 };    //海拔高度
  char GroundLevel[8] = { 0 };    //大地水准面
  char base_length[8] = { 0 };    //基线长度
  char base_dir[8] = { 0 };    //基线测量的航向
  char base_pitch[8] = { 0 };    //基线测的俯仰角
  //char gSpeed[8]       = {0};    //垂直速度
  //char mSpeed[8]       = {0};    //水平速度
  char speedy[10] = { 0 };    //双天线状态,NARROW_INT为可信状态
  char speedx[10] = { 0 };    //双天线状态,NARROW_INT为可信状态
  char speedz[10] = { 0 };    //双天线状态,NARROW_INT为可信状态		
  char dAntenna[10] = { 0 };    //双天线状态,NARROW_INT为可信状态
  char gpsNum = 0;			 //星数局部变量
  char gpsStatue = 0;			//双天线状态,NARROW_INT为可信状态

  const char* revice_data = recvData;

  /*******************************************************GGA语句解析部分**************************************************************/

  str = strstr((const char*)revice_data, (const char*)"GGA,");       //从缓存里找到GGA开头数据，字符串所在地址
  if (str != NULL)                       //不为空则开始继续
  {
    if (CheckGpsData((char*)(str - 3)))  //校验成功
    {
      GPS_data.timeout = 0;
      /************************变量初始化，防止乱码**************************/
      commanum = 0;
      /************************变量初始化，防止乱码**************************/

      /************************解析GGA数据，提取需要的参数*******************/
      do            {
        if ((*str++) == ',')                {
          commanum++;
          BufIndex = 0;
        }
        if (*str != ',')                {
          switch (commanum)                    {

          case 6://状态
            gpsStatue = *str - '0';                                    //状态，1单点定位，4为差分定位
            if (gpsStatue > 9) gpsStatue = 0;
            break;
          case 7://卫星数 
            if (BufIndex < 2)
              gpsNum = gpsNum * 10 + (*str - '0');
            break;
          case 9://天线离海平面高度
            if (BufIndex < sizeof(AltitudeBuf))
              AltitudeBuf[BufIndex] = *str;
            break;
          case 11://大地水准面
            if (BufIndex < sizeof(GroundLevel))
              GroundLevel[BufIndex] = *str;
            break;
          default:
            break;
          }
          if (commanum > 14)
            break;
          BufIndex++;
        }
      }             while (*str != '*');
      /****************************解析GGA数据完成******************************/
      if (commanum == 14)            {
        AltitudeBuf[sizeof(AltitudeBuf) - 1] = 0;
        GroundLevel[sizeof(GroundLevel) - 1] = 0;
        GPS_data.alt = (atof((const char*)AltitudeBuf) + atof((const char*)GroundLevel));         //高度
        GPS_data.gpsStatus = gpsStatue;
        GPS_data.gpsSatNum = gpsNum;                                                                                 //星数
        GPS_data.updata |= 0x01;
      }
    }
  }
  /*******************************************************GGA语句解析完成*************************************************************/


  /*******************************************************RMC语句解析部分**************************************************************/
  str = strstr((const char*)revice_data, (const char*)"RMC,");
  if (str != NULL)    {
    if (CheckGpsData((char*)(str - 3)))        {
      GPS_data.timeout = 0;
      memset(UtcTimeBuf, 0, sizeof(UtcTimeBuf));
      memset(UtcDataBuf, 0, sizeof(UtcDataBuf));
      memset(LatitudeBuf, 0, sizeof(LatitudeBuf));
      memset(LongitudeBuf, 0, sizeof(LongitudeBuf));
      commanum = 0;
      do            {
        if ((*str++) == ',')                {
          commanum++;
          BufIndex = 0;
        }
        if (*str != ',')                {
          switch (commanum)                    {
          case 1://时分秒
            if (BufIndex < sizeof(UtcTimeBuf))
              UtcTimeBuf[BufIndex] = *str;
            break;

          case 3://纬度
            if (BufIndex < (sizeof(LatitudeBuf) - 1))
              LatitudeBuf[1 + BufIndex] = *str;
            break;
          case 4://NS      南纬北纬区别
            LatitudeBuf[0] = *str;
            break;
          case 5://经度
            if (BufIndex < (sizeof(LongitudeBuf) - 1))
              LongitudeBuf[1 + BufIndex] = *str;
            break;
          case 6://EW
            LongitudeBuf[0] = *str;
            break;
          case 9://日月年
            if (BufIndex < sizeof(UtcDataBuf))
              UtcDataBuf[BufIndex] = *str;
            break;
          default:
            break;
          }
          if (commanum > 12)
            break;
          BufIndex++;
        }
      }             while (*str != '*');
      if (commanum == 12)            {
        LatitudeBuf[sizeof(LatitudeBuf) - 1] = 0;
        LongitudeBuf[sizeof(LongitudeBuf) - 1] = 0;
        //            if( UtcDataBuf[0] > 0 )   //yy:mm:dd
        //            {
        //                GPS_DataTYPE.BCDTimeBuf[0] = ( ( UtcDataBuf[4] & 0x0f ) << 4 ) | ( UtcDataBuf[5] & 0x0f );

        //                GPS_DataTYPE.BCDTimeBuf[1] = ( ( UtcDataBuf[2] & 0x0f ) << 4 ) | ( UtcDataBuf[3] & 0x0f );

        //                GPS_DataTYPE.BCDTimeBuf[2] = ( ( UtcDataBuf[0] & 0x0f ) << 4 ) | ( UtcDataBuf[1] & 0x0f );
        //            }

        //            if( UtcTimeBuf[0] > 0 )  //hh:mm:ss
        //            {
        //                GPS_DataTYPE.BCDTimeBuf[3] = ( ( UtcTimeBuf[0] & 0x0f ) << 4 ) | ( UtcTimeBuf[1] & 0x0f );
        //                GPS_DataTYPE.BCDTimeBuf[4] = ( ( UtcTimeBuf[2] & 0x0f ) << 4 ) | ( UtcTimeBuf[3] & 0x0f );
        //                GPS_DataTYPE.BCDTimeBuf[5] = ( ( UtcTimeBuf[4] & 0x0f ) << 4 ) | ( UtcTimeBuf[5] & 0x0f );
        //            }

        //            //UTC ? ????
        //            GPS_DataTYPE.BJ_Time[0] = UtcDataBuf[4], GPS_DataTYPE.BJ_Time[1] = UtcDataBuf[5];
        //            GPS_DataTYPE.BJ_Time[2] = UtcDataBuf[2], GPS_DataTYPE.BJ_Time[3] = UtcDataBuf[3];
        //            GPS_DataTYPE.BJ_Time[4] = UtcDataBuf[0], GPS_DataTYPE.BJ_Time[5] = UtcDataBuf[1];
        //            memcpy( GPS_DataTYPE.BJ_Time + 6, UtcTimeBuf, 6 );
        //            GPS_DataTYPE.BJ_Time[14] = 0;
        ////          UTC_TO_BJ( GPS_DataTYPE.BJ_Time ); //

                        //纬度ddmm.mmmmm-> dd.ddddddd
        if (LatitudeBuf[1] > 0)                {
          commanum = (LatitudeBuf[1] & 0x0f) * 10 + (LatitudeBuf[2] & 0x0f);
          GPS_data.lat = (atof((const char*)(&LatitudeBuf[3])) / 60 + commanum);        //纬度
        }
        //经度转ddmm.mmmmm-> dd.ddddddd
        if (LongitudeBuf[1] > 0)                {
          temp_diff_data.lon = (atof((const char*)(&LongitudeBuf[1])));
          commanum = temp_diff_data.lon / 100;
          GPS_data.lon = ((temp_diff_data.lon - commanum * 100) / 60 + commanum);        //经度
        }
        // GPS_data.tnow = os_time();
        GPS_data.tnow = 0;
        GPS_data.updata |= 0x02;
      }
    }
  }
  /*******************************************************RMC语句解析完成*************************************************************/



  /*******************************************************侧向语句解析部分**************************************************************/
  //str = strstr( ( const char* ) revice_data , ( const char* )  "#DUALANTEN" );
  str = strstr((const char*)revice_data, (const char*)"#HEADINGA");
  if (str != NULL)    {
    if (CheckGpsData_32b((char*)(str + 1)))  //校验成功
//				if( 1 )  //校验成功
    {
      GPS_data.timeout = 0;
      memset(base_length, 0, sizeof(base_length));
      memset(base_dir, 0, sizeof(base_dir));
      memset(base_pitch, 0, sizeof(base_pitch));
      commanum = 0;

      do            {
        if ((*str++) == ',')                {
          commanum++;
          BufIndex = 0;
        }

        if (*str != ',')                {
          switch (commanum)                    {
          case 10://  解算状态，INT为可用
            if (BufIndex < sizeof(dAntenna))                            {
              dAntenna[BufIndex] = *str;
            }
            break;
          case 11://  基线长度
            if (BufIndex < sizeof(base_length))                            {
              base_length[BufIndex] = *str;
            }
            break;
          case 12://  航向角，此处为真正的双天线航向
            if (BufIndex < sizeof(base_dir))                            {
              base_dir[BufIndex] = *str;
            }
            break;


          default:
            break;
          }
          if (commanum > 25)
            break;
          BufIndex++;
        }
      }             while (*str != '*');
      if (commanum == 25)						{

        if (dAntenna[7] == 'I' && dAntenna[8] == 'N' && dAntenna[9] == 'T')  //双天线状态，根据是否_INT，判断可用，只有为INT时双天线计算的航向可用
        {
          GPS_data.dGPS_updata = true;
          GPS_data.dGPS_statue = 4;//航向可用
          base_length[sizeof(base_length) - 1] = 0;
          base_dir[sizeof(base_dir) - 1] = 0;
          base_pitch[sizeof(base_pitch) - 1] = 0;

          if (base_length[0] > 0)									{
            GPS_data.dGPS_base_length = atof((const char*)(base_length));        //基线距离
          }
          if (base_dir[0] > 0)									{
            GPS_data.dGPS_base_dir = atof((const char*)(&base_dir));             //双天线航向角

            GPS_data.dGPS_base_dir = _wrap_180(GPS_data.dGPS_base_dir + 90 - 0.3f + 0.33f);  // 安装偏差0.71f+0.1741f

              /* 时间间隔
              static uint32_t ekf_time_Last=0;
              uint32_t ekf_time_Now=HAL_GetTick();
              float ekf_dt=(ekf_time_Now - ekf_time_Last)*0.001f;
              ekf_time_Last=ekf_time_Now;

              printf("%f\r\n",ekf_dt);   */
          }
          if (base_pitch[0] > 0)									{
            GPS_data.dGPS_base_pitch = atof((const char*)(&base_pitch));          //双天线俯仰角
          }
          GPS_data.updata |= 0x04;
          //get_normal_sensor(gps_diff , 0);
        }
        else							{
          GPS_data.dGPS_updata = false;
          GPS_data.dGPS_base_length = 0;
          GPS_data.dGPS_base_dir = 0;
          GPS_data.dGPS_base_pitch = 0;
          GPS_data.dGPS_statue = 0;//航向不可用	
        }
      }
    }
  }
  /*******************************************************测向语句解析完成**************************************************************/



  /*******************************************************测速语句解析部分**************************************************************/
  str = strstr((const char*)revice_data, (const char*)"#BESTXYZA");
  if (str != NULL)    {
    if (CheckGpsData_32b((char*)(str + 1)))  //校验成功
//        if(1)      //暂未判断校验和//if( CheckGpsData( ( char* )( str - 3 ) ) == 1 )
    {
      GPS_data.timeout = 0;
      commanum = 0;

      do            {
        if ((*str++) == ',')                {
          commanum++;
          BufIndex = 0;
        }

        if (*str != ',')                {
          switch (commanum)                    {

          case 19://wgs84 X轴速度
            if (BufIndex < sizeof(speedx))                            {
              speedx[BufIndex] = *str;
            }
            break;
          case 20://wgs84 y轴速度
            if (BufIndex < sizeof(speedy))                            {
              speedy[BufIndex] = *str;
            }
            break;
          case 21://垂直速度
            if (BufIndex < sizeof(speedz))                            {
              speedz[BufIndex] = *str;
            }
            break;
          default:
            break;
          }
          if (commanum > 35)
            break;
          BufIndex++;
        }
      }             while (*str != '*');
      if (commanum == 36)            {
        speedx[sizeof(speedx) - 1] = 0;
        speedy[sizeof(speedy) - 1] = 0;
        speedz[sizeof(speedz) - 1] = 0;
        if (speedx[0] > 0)temp_diff_data.SpeedE = atof((const char*)(&speedx)); //水平速度  m/s  
        if (speedy[0] > 0)temp_diff_data.SpeedN = atof((const char*)(&speedy)); //水平速度  m/s  	
        if (speedz[0] > 0)temp_diff_data.SpeedU = atof((const char*)(&speedz)); //垂直速度 m/s     

        GPS_data.SpeedE = -sin(GPS_data.lon * 0.0175f) * temp_diff_data.SpeedE + cos(GPS_data.lon * 0.0175f) * temp_diff_data.SpeedN;
        GPS_data.SpeedN = -sin(GPS_data.lat * 0.0175f) * cos(GPS_data.lon * 0.0175f) * temp_diff_data.SpeedE - \
          sin(GPS_data.lon * 0.0175f) * sin(GPS_data.lat * 0.0175f) * temp_diff_data.SpeedN + cos(GPS_data.lat * 0.0175f) * temp_diff_data.SpeedU;
        GPS_data.SpeedU = cos(GPS_data.lon * 0.0175f) * cos(GPS_data.lat * 0.0175f) * temp_diff_data.SpeedE + \
          cos(GPS_data.lat * 0.0175f) * sin(GPS_data.lon * 0.0175f) * temp_diff_data.SpeedN + sin(GPS_data.lat * 0.0175f) * temp_diff_data.SpeedU;
        GPS_data.updata |= 0x08;              //防止同一组数据重复进入	
      }
    }
  }
  /*******************************************************测速语句解析完成*************************************************************/

//    if((temp_diff_data.updata  & 0x0f) == 0x0f)
//    {
//        GPS_data.updata    = true;
//        GPS_data  = temp_diff_data;
//        //get_normal_sensor(gps_diff_pos , 0);               //判断数据是否异常函数，暂未添加内容，可屏蔽
//        memset(&temp_diff_data, 0, sizeof( temp_diff_data) );
//    }


  return 0;

}

/*
函数功能：解析下位机向上位机发送的数据
函数参数：
      buff:数据包首地址
返回值：
    当前的数据包的命令类型
      DATAPACKTtype     数据包
*/
void RecieveLocData(){
  static unsigned char buff[500];
  static unsigned char LastRecNum = 0;
// 	char tempbuff[] = "#BESTXYZA,COM1,0,97.0,FINE,2175,552833.100,13071,6,18;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0.000,0.000,2.700,18,0,0,1,0,00,0,00*8c5e59bf $GNGGA,093335.10,,,,,0,00,9999.0,,,,,,*46 $GNRMC,093335.10,V,,,,,,,180921,0.0,E,N*05 #HEADINGA,COM1,0,97.0,FINE,2175,552833.100,13071,7,18;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*11083efa";
// 	    int statusflag = RecePro((char*)tempbuff); //解析下位机向上位机发送的数据包
// 	    printf("\n>>>> ==========GPS Data===========\nlat:%f\tlon:%f\tgpsStatus:%d\nalt:%f\ttimeout:%d\n", GPS_data.lat, GPS_data.lon, GPS_data.gpsStatus, GPS_data.alt, GPS_data.timeout);

// return;
  int nfds = epoll_wait(serial_obj.epfd, serial_obj.events, 20, 0);

  if (nfds > 0)	{
    memset(buff, 0, sizeof(buff));
//    int num = read(serial_obj.MsgCenterSocket, &buff[LastRecNum], 415);
     int num = read(serial_obj.MsgCenterSocket, &buff[0], 2);
//    tcflush(serial_obj.MsgCenterSocket, TCIOFLUSH);
    if (((buff[0] == '#') && (buff[1] == 'B'))){
     	    num = read(serial_obj.MsgCenterSocket, &buff[2], 462 - 2);
	    int statusflag = RecePro((char*)buff); //解析下位机向上位机发送的数据包
	    printf("\n==========GPS Data===========\nlat:%f\tlon:%f\tgpsStatus:%d\nalt:%f\ttimeout:%d\n", GPS_data.lat, GPS_data.lon, GPS_data.gpsStatus, GPS_data.alt, GPS_data.timeout);
	    GPS_pub.publish(GPS_data);
    }
  }
}

void* GetConnetDataFun(void* p){
	printf("Start Get GPS data\n");
  tcflush(serial_obj.MsgCenterSocket, TCIOFLUSH);
  while (1)	{
    RecieveLocData();
  }
}

//主函数
int main(int argc, char** argv)
{
  static int len;
  static int len_total;

  if(signal(SIGINT, sig_handler) == SIG_ERR){//signal catch
  	printf("Can't catch SIGINT\n");
	exit(0);
  }
  //初始化节点
  ros::init(argc, argv, "gpspub_node");
  //声明节点句柄
  ros::NodeHandle nh;
  //注册Publisher到话题GPS
  GPS_pub = nh.advertise<gpspub::GPS>("GPS",1000);

  memset(&serial_obj, 0, sizeof(serial_obj));

  //socket套接字的客户端连接
  serial_obj.MsgCenterSocket = open(GPS_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_obj.MsgCenterSocket <= 0)    {
    printf("Fail to open %s!\n", GPS_PORT);
    return -1;
  }

  int ret = UART_Init_Baud(serial_obj.MsgCenterSocket, 115200);
  if (ret != 0){
        printf("Init UART failed\n");
        return 0;
  }


  serial_obj.epfd = epoll_create(100);//创建epoll句柄 

  serial_obj.ev.data.fd = serial_obj.MsgCenterSocket;
  //设置要处理的事件类型  
  serial_obj.ev.events = EPOLLIN;
  //注册epoll事件  
  epoll_ctl(serial_obj.epfd, EPOLL_CTL_ADD, serial_obj.MsgCenterSocket, &serial_obj.ev);

  pthread_t connect_pth;
  pthread_create(&connect_pth, NULL, GetConnetDataFun, NULL);
  pthread_join(connect_pth, NULL);

  return 0;
}


