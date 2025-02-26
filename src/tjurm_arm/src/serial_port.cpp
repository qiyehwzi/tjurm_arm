#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <cstring>
#include "serial_port.h"
 
uint8_t rx_buffer[24];
uint8_t tx_buffer[24];
float rx_buffer_float[6];
float tx_buffer_float[6];

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    
    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
    
    try
    {
        //打开串口
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
    
    ros::Rate loop_rate(500);
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
        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;
}
