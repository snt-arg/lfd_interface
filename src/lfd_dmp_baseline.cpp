

#include <lfd_interface/lfd_pipeline.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>


int main(int argc, char** argv)
{
    ros::init(argc,argv,"lfd_dmp_baseline");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string planning_group,base_frame;
    std::size_t error = 0, demo_count;
    std::string LOGNAME{"lfd_dmp_baseline"};
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "planning_group", planning_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "base_frame", base_frame);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    LFDPipeline lfd_pipeline(planning_group, base_frame);


    ros::waitForShutdown();

}
