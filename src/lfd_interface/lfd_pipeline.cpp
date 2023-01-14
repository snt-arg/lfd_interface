#include <lfd_interface/lfd_pipeline.h>


LFDPipeline::LFDPipeline(std::string planning_group,std::string base_frame):
moveit_util_(planning_group,base_frame), lfd_planner_(moveit_util_), lfd_controller_(moveit_util_),
as_lfd_pipeline_(nh_, "lfd_pipeline", boost::bind(&LFDPipeline::executeCBLFDPipeline, this, _1), false)
{
    sc_democount_ = nh_.serviceClient<lfd_interface::DemoCount>("fetch_demo_count");
    as_lfd_pipeline_.start();
}

LFDPipeline::~LFDPipeline()
{
}

void LFDPipeline::executeCBLFDPipeline(const lfd_interface::LFDPipelineGoalConstPtr &goal)
{
    lfd_interface::DemoCount srv;
    srv.request.name = goal->name;
    
    sc_democount_.call(srv);

    int demo_count = srv.response.count;

    if (goal->train)
    {
        ROS_INFO("Training Started");

        for (size_t i = 0; i < demo_count; i++)
        {
            lfd_trainer_.init(goal->name + std::to_string(i));
            lfd_trainer_.run();
        }
        ROS_INFO("Training Finished");
    }
    if (goal->visualize)
    {
        for (size_t i = 0; i < demo_count; i++)
        {
            lfd_planner_.init(goal->name + std::to_string(i));
            lfd_planner_.runViz();
        }
    }
    else if (goal->execute)
    {
        for (size_t i = 0; i < demo_count; i++)
        {
            lfd_planner_.init(goal->name + std::to_string(i));
            lfd_planner_.runExec();
        }
    }
    else if (goal->control)
    {
        for (size_t i = 0; i < demo_count; i++)
        {
            lfd_planner_.init(goal->name + std::to_string(i));
            lfd_planner_.visualizePlannedTrajectory();
            lfd_controller_.runControl(lfd_planner_.fetchPlanMetaData());
        }
    }

    result_lfd_pipeline_.success = true;
    as_lfd_pipeline_.setSucceeded(result_lfd_pipeline_);
}