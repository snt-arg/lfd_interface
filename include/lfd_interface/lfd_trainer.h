#pragma once

//ROS
#include "ros/ros.h"

//LFD
#include <lfd_interface/DemonstrationMsg.h>
#include <lfd_interface/GetDemonstration.h>
#include <lfd_interface/TrainDemonstration.h>

class LFDTrainer
{
private:
    const std::string LOGNAME{"lfd_trainer"};

    ros::NodeHandle nh_;
    //name of the trajectory to be loaded
    std::string demonstration_name_;

    //ros stuff
    ros::ServiceClient client_load_demonstration_, client_train_demonstration_;
    lfd_interface::GetDemonstration srv_get_demonstration_;
    lfd_interface::TrainDemonstration srv_train_demonstration_;
    lfd_interface::DemonstrationMsg demonstration_;

private:
    bool loadDemonstration();
    bool trainDemonstration();

public:
    LFDTrainer(std::string robot_ns);
    ~LFDTrainer();
    void run();
    void init(std::string demonstration_name);
    lfd_interface::DemonstrationMsg fetchDemonstration();
};
