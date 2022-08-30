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


    enum rviz_visual_tools::colors active_color_ = rviz_visual_tools::LIME_GREEN;

    MoveGroupPtr getMoveGroup();
    VisualToolsPtr getVisualTools();

    void publishText(std::string text);
    void currentJointState(trajectory_msgs::JointTrajectoryPoint & joint_values);
    void visualizeJointTrajectory(trajectory_msgs::JointTrajectory & trajectory);
    void changeVisualizationColor(enum rviz_visual_tools::colors color);
};
