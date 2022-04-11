#pragma once

//ROS
#include "ros/ros.h"

#include <lfd_interface/lfd_trainer.h>

//LFD
#include <lfd_interface/DemonstrationMsg.h>
#include <lfd_interface/PlanLFD.h>

#include <lfd_interface/moveit_util.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

class LFDPlanner
{
private:
    const std::string LOGNAME{"lfd_planner"};

    ros::NodeHandle nh_;
    //name of the trained demonstration to use for planning
    std::string demonstration_name_;

    MoveitUtil& moveit_util_;
    //just for testing
    LFDTrainer trainer;
    lfd_interface::DemonstrationMsg demonstration_;

    ros::ServiceClient client_plan_lfd_;
    trajectory_msgs::JointTrajectory plan_;

    ros::Publisher pub_displayplan_;
private:

    void getPlan(trajectory_msgs::JointTrajectoryPoint start,
                trajectory_msgs::JointTrajectoryPoint goal);
    void refine();
    void displayPlannedPath();

    void executePlan();

public:
    LFDPlanner(std::string demonstration_name, MoveitUtil & moveit_util);
    ~LFDPlanner();
    void run();
};

