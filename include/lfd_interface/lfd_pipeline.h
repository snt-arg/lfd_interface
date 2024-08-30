#pragma once

#include <lfd_interface/lfd_recorder.h>
#include <lfd_interface/lfd_trainer.h>
#include <lfd_interface/lfd_planner.h>
#include <lfd_interface/lfd_controller.h>

#include <lfd_interface/DemoCount.h>
#include <lfd_interface/LFDPipelineAction.h>

#include <actionlib/server/simple_action_server.h>

class LFDPipeline
{

private:
    ros::NodeHandle nh_;
    ros::ServiceClient sc_democount_;
    MoveitUtil moveit_util_;
    LFDTrainer lfd_trainer_;
    LFDPlanner lfd_planner_;
    LFDController lfd_controller_;

    actionlib::SimpleActionServer<lfd_interface::LFDPipelineAction> as_lfd_pipeline_;
    lfd_interface::LFDPipelineResult result_lfd_pipeline_;

public:
    LFDPipeline(std::string planning_group,std::string base_frame, std::string robot_ns); ;
    ~LFDPipeline();


private:
    void executeCBLFDPipeline(const lfd_interface::LFDPipelineGoalConstPtr &goal);
};
