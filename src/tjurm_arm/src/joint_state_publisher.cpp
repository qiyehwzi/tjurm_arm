#include <ros/ros.h>
#include <sensor_msgs/JointState.h> 
#include <serial/serial.h>
#include <iostream>
#include <cstring>

# include <actionlib/server/simple_action_server.h>
# include <control_msgs/FollowJointTrajectoryAction.h>
# include <std_msgs/Float32MultiArray.h>
# include <moveit_msgs/RobotTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
 
serial::Serial sp;  //创建一个serial类
sensor_msgs::JointState joint_state;    // 定义关节状态消息
uint8_t rx_buffer[24];
uint8_t tx_buffer[24];
float rx_buffer_float[6];
float tx_buffer_float[6];

void serial_init();

// 重命名类型为 Server
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

// 用于存储 moveit 发送出来的轨迹数据
moveit_msgs::RobotTrajectory moveit_tra;

int n_joints;
int n_tra_Points;

void execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goalPtr, Server* moveit_server)
{
    // 1、解析提交的目标值
    n_joints = goalPtr->trajectory.joint_names.size();
    n_tra_Points = goalPtr->trajectory.points.size();
    
    moveit_tra.joint_trajectory.header.frame_id = goalPtr->trajectory.header.frame_id;
    moveit_tra.joint_trajectory.joint_names = goalPtr->trajectory.joint_names;
    moveit_tra.joint_trajectory.points.resize(n_tra_Points);

    for(int i=0; i<n_tra_Points; i++) // 遍历每组路点
    {
        moveit_tra.joint_trajectory.points[i].positions.resize(n_joints);
        moveit_tra.joint_trajectory.points[i].velocities.resize(n_joints);
        moveit_tra.joint_trajectory.points[i].accelerations.resize(n_joints);
        
        moveit_tra.joint_trajectory.points[i].time_from_start = goalPtr->trajectory.points[i].time_from_start;
        for(int j=0;j<n_joints; j++) // 遍历每组路点中的每个关节数据
        {
            moveit_tra.joint_trajectory.points[i].positions[j] = goalPtr->trajectory.points[i].positions[j];
            moveit_tra.joint_trajectory.points[i].velocities[j] = goalPtr->trajectory.points[i].velocities[j];
            moveit_tra.joint_trajectory.points[i].accelerations[j] = goalPtr->trajectory.points[i].accelerations[j];
        }
    }

    std::cout << "The trajectory data is:" << "********************************************" << std::endl;
    std::cout << moveit_tra;
    std::cout << "********************************************" << "The trajectory data is finished printing." << std::endl;
    ROS_INFO("The number of joints is %d.",n_joints);
    ROS_INFO("The waypoints number of the trajectory is %d.",n_tra_Points);

    ROS_INFO("Receive trajectory successfully");
    moveit_server->setSucceeded();
}


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

    // // 创建异步spinner
    // ros::AsyncSpinner spinner(1); // 使用1个线程
    // spinner.start();
    // // 初始化MoveIt!接口
    // moveit::planning_interface::MoveGroupInterface move_group("arm");

    // 创建 action 对象(NodeHandle，话题名称，回调函数解析传入的目标值，服务器是否自启动)
    Server moveit_server(nh,"controller/follow_joint_trajectory", boost::bind(&execute_callback, _1, &moveit_server), false);
    // 手动启动服务器
    moveit_server.start();


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

    ros::spin();

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

