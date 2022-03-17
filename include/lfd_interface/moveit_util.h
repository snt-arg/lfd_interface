#pragma once

//ROS
#include "ros/ros.h"

//MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


class MoveitUtil
{
    typedef std::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
    typedef std::shared_ptr<moveit_visual_tools::MoveItVisualTools> VisualToolsPtr;

private:
    const std::string LOGNAME{"moveit_util"};

    MoveGroupPtr move_group_;
    VisualToolsPtr visual_tools_;
    const moveit::core::JointModelGroup* joint_model_group_;

public:
    MoveitUtil(std::string planning_group, std::string viz_base_frame);
    ~MoveitUtil();

    MoveGroupPtr getMoveGroup();
    VisualToolsPtr getVisualTools();

    void publishText(std::string text);
    trajectory_msgs::JointTrajectoryPoint currentJointState();
    void visualizeJointTrajectory(trajectory_msgs::JointTrajectory & trajectory);
};
