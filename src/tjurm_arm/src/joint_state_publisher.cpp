#include <ros/ros.h>
#include <sensor_msgs/JointState.h> 
#include <serial/serial.h>
#include <iostream>
#include <cstring>
 
serial::Serial sp;  //创建一个serial类
sensor_msgs::JointState joint_state;    // 定义关节状态消息
uint8_t rx_buffer[24];
uint8_t tx_buffer[24];
float rx_buffer_float[6];
float tx_buffer_float[6];

void serial_init();

int main(int argc, char *argv[]) {

    ////multithread
    // ros::MultiThdfi  spinner(2);
    // spinner.spin();

    // 初始化 ROS 节点
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;

    // 创建一个发布者，发布到 /joint_states 话题
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ROS_INFO("joint_state_publisher started");

    serial_init();

    //打开串口
    try
    {
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};  // 关节名称
    joint_state.position.resize(6);  // 关节位置数组
    joint_state.velocity.resize(6);  // 关节速度数组
    joint_state.effort.resize(6);    // 关节力矩数组

    // 设置发布频率
    ros::Rate rate(500.0);
    
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            //读出数据
            n = sp.read(rx_buffer, n)/4;

            tx_buffer_float[0] = 3.1415;
            tx_buffer_float[1] = 2.4287;
            tx_buffer_float[2] = 2.71828;
            tx_buffer_float[3] = 0.983;
            tx_buffer_float[4] = 1.895;
            tx_buffer_float[5] = 1.895;

            memcpy(rx_buffer_float,rx_buffer,sizeof(rx_buffer));
            memcpy(tx_buffer,tx_buffer_float,sizeof(tx_buffer_float));
            
            for(int i=0; i<n; i++)
            {
                // std::cout << std::hex << (rx_buffer[i] & 0xff) << " ";
                std::cout << (rx_buffer_float[i]) << " ";
            }
            std::cout << std::endl;
            //把数据发送回去
            // sp.write(rx_buffer, n*4);
            sp.write(tx_buffer, 24);
        }
        
        for (int i = 0; i < 6; ++i) {
            //模拟关节角度
            //joint_state.position[i] = static_cast<double>(rand()) / RAND_MAX * 2.0 - 1.0;  // 随机角度 [-1, 1]

            //stm32 publish real angle
            joint_state.position[i] = rx_buffer_float[i];
            std::cout << rx_buffer_float[i] << " ";
        }

        // 填充时间戳
        joint_state.header.stamp = ros::Time::now();

        // 发布消息
        joint_states_pub.publish(joint_state);

        // 保持循环频率
        rate.sleep();
    }
    
    //关闭串口
    sp.close();

    return 0;
}

void serial_init()
{
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
}

