#pragma once

//ROS
#include "ros/ros.h"

#include <lfd_interface/lfd_trainer.h>

//LFD
#include <lfd_interface/DemonstrationMsg.h>
#include <lfd_interface/PlanLFD.h>
#include <lfd_interface/PlanMsg.h>

#include <lfd_interface/moveit_util.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

class LFDPlanner
{
private:
    const std::string LOGNAME{"lfd_planner"};

    ros::NodeHandle nh_;
    //name of the trained demonstration to use for planning
    std::string demonstration_name_;
    double duration_;

    MoveitUtil& moveit_util_;
    //just for testing
    LFDTrainer trainer;
    lfd_interface::DemonstrationMsg demonstration_;

    ros::ServiceClient client_plan_lfd_;
    trajectory_msgs::JointTrajectory plan_;
    trajectory_msgs::JointTrajectoryPoint goal_joint_;

    ros::Publisher pub_displayplan_;

    lfd_interface::PlanMsg plan_metadata_;

private:

    void getPlan(trajectory_msgs::JointTrajectoryPoint start,
                trajectory_msgs::JointTrajectoryPoint goal);
    void refine();
    void displayPlannedPath();

public:
    LFDPlanner(MoveitUtil & moveit_util, std::string robot_ns);
    ~LFDPlanner();
    void visualizePlannedTrajectory();
    lfd_interface::PlanMsg fetchPlanMetaData();
    trajectory_msgs::JointTrajectory runViz();
    trajectory_msgs::JointTrajectory runExec();
    void init(std::string demonstration_name,
            trajectory_msgs::JointTrajectoryPoint goal_joint=trajectory_msgs::JointTrajectoryPoint(),
            double duration=0.0);
    void executePlan();
};

