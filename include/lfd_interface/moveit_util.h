#pragma once

//ROS
#include "ros/ros.h"

//MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

#include <lfd_interface/PoseTrajectory.h>
#include <lfd_interface/PoseTrajectoryPoint.h>

class MoveitUtil
{
    typedef std::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
    typedef std::shared_ptr<moveit_visual_tools::MoveItVisualTools> VisualToolsPtr;

private:
    const std::string LOGNAME{"moveit_util"};

    MoveGroupPtr move_group_;
    VisualToolsPtr visual_tools_;
    const moveit::core::JointModelGroup* joint_model_group_;

    robot_model_loader::RobotModelLoader robot_model_loader_;
    const moveit::core::RobotModelPtr& kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;

public:
    MoveitUtil(std::string planning_group, std::string viz_base_frame);
    ~MoveitUtil();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    enum rviz_visual_tools::colors active_color_ = rviz_visual_tools::LIME_GREEN;

    MoveGroupPtr getMoveGroup();
    VisualToolsPtr getVisualTools();

    void publishText(std::string text);
    void currentJointState(trajectory_msgs::JointTrajectoryPoint & joint_values);
    void visualizeJointTrajectory(trajectory_msgs::JointTrajectory & trajectory);
    void changeVisualizationColor(enum rviz_visual_tools::colors color);
    lfd_interface::PoseTrajectoryPoint jointToPose(trajectory_msgs::JointTrajectoryPoint & joint_states);
    void currentPose(lfd_interface::PoseTrajectoryPoint & pose_values);
    void visualizePosePath(lfd_interface::PoseTrajectory & trajectory);
    void planPath(std::vector<double> joint_positions);
    void move();
};
