#pragma once

//ROS
#include "ros/ros.h"

//LFD
#include <lfd_interface/DemonstrationMsg.h>
#include <lfd_interface/moveit_util.h>

class LFDRecorder
{
private:
    const std::string LOGNAME{"lfd_recorder"};
    const double JOINTMOVE_THRESH =  std::numeric_limits<float>::epsilon();
    const int RECORDER_LOOP_RATE = 20;

    ros::NodeHandle nh_;

    MoveitUtil& moveit_util_;

    //recorder variables
    lfd_interface::DemonstrationMsg demonstration_;

    //ros stuff
    ros::Publisher pub_save_demonstration_;

private:

    void saveDemonstration();
    bool robotHasMoved(const trajectory_msgs::JointTrajectoryPoint & newstate);

public:
    LFDRecorder(MoveitUtil & moveit_util);
    ~LFDRecorder();
    void run(std::string demonstration_name);
};
