

#include <lfd_interface/lfd_recorder.h>
#include <lfd_interface/lfd_trainer.h>
#include <lfd_interface/lfd_planner.h>
#include <lfd_interface/lfd_controller.h>
#include <lfd_interface/DemoCount.h>

#include <lfd_interface/LFDPipelineAction.h>

#include <actionlib/server/simple_action_server.h>


#include <rosparam_shortcuts/rosparam_shortcuts.h>

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
    LFDPipeline(std::string planning_group,std::string base_frame) ;
    ~LFDPipeline();


private:
    void executeCBLFDPipeline(const lfd_interface::LFDPipelineGoalConstPtr &goal);
};

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

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lfd_dmp_baseline");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string demo_namebase,planning_group,base_frame, demo_name;
    std::size_t error = 0, demo_count;
    std::string LOGNAME{"lfd_dmp_baseline"};
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demonstration_name", demo_namebase);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "planning_group", planning_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "base_frame", base_frame);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demo_count", demo_count);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    LFDPipeline lfd_pipeline(planning_group, base_frame);


    ros::waitForShutdown();

}
