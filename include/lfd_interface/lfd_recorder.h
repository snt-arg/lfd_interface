#pragma once

//ROS
#include "ros/ros.h"


//MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

//LFD
#include <lfd_interface/DemonstrationMsg.h>
#include <lfd_interface/moveit_util.h>

class LFDRecorder
{
private:
    const std::string LOGNAME{"lfd_recorder"};

    ros::NodeHandle nh_;

    MoveitUtil& moveit_util_;
    //recorder variables
    lfd_interface::DemonstrationMsg demonstration_;

    //ros stuff
    ros::Publisher pub_save_demonstration_;

private:

    void saveDemonstration();

public:
    LFDRecorder(MoveitUtil & moveit_util);
    ~LFDRecorder();
    void run(std::string demonstration_name);
};
