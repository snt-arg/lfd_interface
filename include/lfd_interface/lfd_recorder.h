#pragma once

//ROS
#include "ros/ros.h"
#include <rosparam_shortcuts/rosparam_shortcuts.h>

//MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

//LFD
#include <lfd_interface/DemonstrationMsg.h>

class LFDRecorder
{
private:
    const std::string LOGNAME{"lfd_recorder"};

    ros::NodeHandle pnh_;
    ros::NodeHandle nh_;
    //name of the trajectory to be recorded
    std::string demonstration_name_;


    //moveit planning
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    const moveit::core::JointModelGroup* joint_model_group_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;

    //recorder variables
    lfd_interface::DemonstrationMsg demonstration_;

    //ros stuff
    ros::Publisher pub_save_demonstration_;

private:

    void publishText(std::string text);
    trajectory_msgs::JointTrajectoryPoint currentJointState();
    void visualizeTrajectory();
    void saveDemonstration();

public:
    LFDRecorder(std::string demonstration_name, std::string planning_group, std::string base_frame,ros::NodeHandle& pnh);
    ~LFDRecorder();
    void run();
};
