# include <ros/ros.h>
# include <actionlib/server/simple_action_server.h>
# include <control_msgs/FollowJointTrajectoryAction.h>
# include <std_msgs/Float32MultiArray.h>
# include <iostream>
# include <moveit_msgs/RobotTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


// 重命名类型为 Server
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

// 用于存储 moveit 发送出来的轨迹数据
moveit_msgs::RobotTrajectory moveit_tra;

void execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goalPtr, Server* moveit_server)
{
    // 1、解析提交的目标值
    int n_joints = goalPtr->trajectory.joint_names.size();
    int n_tra_Points = goalPtr->trajectory.points.size();
    
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


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"moveit_action_server");
    ros::NodeHandle nh;

    ROS_INFO("action server started");

    // // 创建异步spinner
    // ros::AsyncSpinner spinner(1); // 使用1个线程
    // spinner.start();
    // // 初始化MoveIt!接口
    // moveit::planning_interface::MoveGroupInterface move_group("arm");

    // 创建 action 对象(NodeHandle，话题名称，回调函数解析传入的目标值，服务器是否自启动)
    Server moveit_server(nh,"controller/follow_joint_trajectory", boost::bind(&execute_callback, _1, &moveit_server), false);
    // 手动启动服务器
    moveit_server.start();

    ros::spin();
    return 0;
}

