/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-17 15:50:38
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-18 09:45:17
 */
#include <ros/ros.h> 
#include <stdio.h>
#include <std_msgs/String.h>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <sstream>
#include "../include/datapack.h"
#include "../include/kbhit.h"

#define PUB_TOPIC   "send_to_car"

#define FORWARD     65
#define BACKWARD    66
#define RIHGT       67
#define LEFT        68

#define NOKEY       0

int GetDirection(void){
    static char keyD[3] = { 0 };
    int key = scanKeyboard();
    if (key == 27){
        keyD[0] = key;
    }
    else if (key == 91){
        keyD[1] = key;
    }

    if (keyD[0] == 27 && keyD[1] == 91 && (key >= 65 && key <= 68)){
        keyD[2] = key;
        if (keyD[2] == FORWARD){
            printf("\n↑\n");
        }
        else if (keyD[2] == BACKWARD){
            printf("\n↓\n");
        }
        else if (keyD[2] == RIHGT){
            printf("\n→\n");
        }
        else if (keyD[2] == LEFT){
            printf("\n←\n");
        }
        return key;
    }

    return NOKEY;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "control_node");       //a)
    ros::NodeHandle n;                           //b)
    ros::Publisher control_pub = n.advertise<std_msgs::String>(PUB_TOPIC, 1000);
    
  /* ros::Rate 对象可以允许你指定自循环的频率。它会追踪记录自上一次调用 Rate::sleep() 后时间的流逝，并休眠直到一个频率周期的时间。在这个例子中，让它以 10Hz 的频率运行，即节点休眠时间为100ms。 */
    ros::Rate loop_rate(100); // Hz
    
    while (ros::ok()){
        /*进入节点的主循环，如果下列条件之一发生，ros::ok() 返回false，跳出循环：
    
    ·SIGINT 被触发 (Ctrl+C)：roscpp 会默认生成一个 SIGINT 句柄，它负责·处理 Ctrl+C 键盘操作使ros::ok() 返回 false
    ·被另一同名节点踢出 ROS 网络
    ·关闭函数ros::shutdown() 被程序的另一部分调用
    ·节点中的所有 ros::NodeHandles 都已经被销毁
        一旦 ros::ok() 返回 false, 所有的 ROS 调用都会失效。
        */
        int dir = GetDirection();
        if (dir >= FORWARD && dir <= LEFT){
            char senddata[11] = {0};
            std_msgs::String Data;

            if (dir == FORWARD){
                ControlDataPack(senddata, 200, 0, 0);
            }
            else if (dir == BACKWARD){
                ControlDataPack(senddata, -200, 0, 0);
            }
            else if (dir == LEFT){
                ControlDataPack(senddata, 0, 0, 200);
            }
            else if (dir == RIHGT){
                ControlDataPack(senddata, 0, 0, -200);
            }
            Data.data = senddata;
            control_pub.publish(Data);
        }


        //std_msgs::String msg;//建立暂存区，先将消息放入，在进行publish
       // control_pub.publish(msg);//发布封装完毕的消息msg。Master会查找订阅该话题的节点，并完成两个节点的连接，传输消息

        ros::spinOnce();//处理订阅话题的所有回调函数callback()，
        loop_rate.sleep(); //休眠，休眠时间由loop_rate()设定

    }
    
}