/*
 * @Description: 
 * @Version: 
 * @Autor: Zeng Tianhao
 * @Date: 2021-09-02 13:48:07
 * @LastEditors: Zeng Tianhao
 * @LastEditTime: 2021-09-03 10:12:35
 */
//头文件部分
#include <sstream> 
#include "ros/ros.h"
 /*"ros/ros.h 是一个实用的头文件，它引用了 ROS 系统中大部分常用的头文件。"
地址在 /opt/ros/noetic/include/ros/" */
#include "std_msgs/String.h" 
/* "std_msgs"是一个消息类型依赖包，此处要传输string类型数据，
需要包含该数据类型的头文件String.h，就在这个依赖包里 */

//初始化部分
int main(int argc, char** argv){
    ros::init(argc, argv, "testpub");

    ros::NodeHandle n;
    /* 为这个进程的节点创建一个句柄。
    第一个创建的 NodeHandle 会为节点进行初始化，
    最后一个销毁的 NodeHandle 则会释放该节点所占用的所有资源。 */

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("message", 1000);
    /* 告诉 master 将要在 chatter（话题名）上发布 std_msgs/String 消息类型的消息。
    这样 master 就会告诉所有订阅了 chatter 话题的节点，将要有数据发布。
    NodeHandle::advertise() 会建立一个topic。在ROS Master端注册一个Publisher，
    返回一个 ros::Publisher 对象,此处为chatter_pub，,它有两个作用：
    1) 它有一个 publish()函数可以在topic上发布(pubish)消息；
    2) <std_msgs::String>指定后面要发布的消息类型是std_msgs包中的string类型，如果消息类型不对,它会拒绝发布。

    ("chatter", 1000)中两个参数，第一个参数指定以"chatter"为话题发布消息
    第二个参数是发布序列的大小。如果发布的消息的频率太高，缓冲区中的消息在大于 1000 个的时候就会开始丢弃先前发布的消息。*/

    ros::Rate loop_rate(0.5); // Hz
    /* ros::Rate 对象可以允许你指定自循环的频率。它会追踪记录自上一次调用 Rate::sleep() 后时间的流逝，并休眠直到一个频率周期的时间。在这个例子中，让它以 10Hz 的频率运行，即节点休眠时间为100ms。 */

    //循环部分
    int count = 0, move = 0;
    while (ros::ok())    {
        /*进入节点的主循环，如果下列条件之一发生，ros::ok() 返回false，跳出循环：

    ·SIGINT 被触发 (Ctrl+C)：roscpp 会默认生成一个 SIGINT 句柄，它负责·处理 Ctrl+C 键盘操作使ros::ok() 返回 false
    ·被另一同名节点踢出 ROS 网络
    ·关闭函数ros::shutdown() 被程序的另一部分调用
    ·节点中的所有 ros::NodeHandles 都已经被销毁
        一旦 ros::ok() 返回 false, 所有的 ROS 调用都会失效。
        */

        std_msgs::String msg;//建立暂存区，先将消息放入，在进行publish
        // sprintf(msg.data.c_str(), "move:%d", move++);
        std::stringstream ss;
        ss << "move:" << move;
        move = !move;
        msg.data = ss.str();//将要输出的字符串消息存储到string消息类型中唯一成员data中
        ROS_INFO("%s", msg.data.c_str()); //类似C/C++的 printf/cout 等函数，打印日志信息。
        chatter_pub.publish(msg);//发布封装完毕的消息msg。Master会查找订阅该话题的节点，并完成两个节点的连接，传输消息

        ros::spinOnce();//处理订阅话题的所有回调函数callback()，
        loop_rate.sleep(); //休眠，休眠时间由loop_rate()设定
        ++count;

    }
    return 0;

}
