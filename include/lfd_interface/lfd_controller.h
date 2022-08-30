#pragma once

//ROS
#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//LFD
#include <lfd_interface/DemonstrationMsg.h>
#include <lfd_interface/PlanLFD.h>
#include <lfd_interface/ControlLFDAction.h>
#include <lfd_interface/PlanMsg.h>

#include <lfd_interface/moveit_util.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>

class LFDController
{
private:
    const std::string LOGNAME{"lfd_controller"};

    ros::NodeHandle nh_;

    MoveitUtil& moveit_util_;

    actionlib::SimpleActionClient<lfd_interface::ControlLFDAction> ac_control_;


public:
    LFDController(MoveitUtil & moveit_util);
    ~LFDController();
    void runControl(lfd_interface::PlanMsg plan_metadata);
};

