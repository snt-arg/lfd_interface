#include <lfd_interface/lfd_recorder.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>


int main(int argc, char** argv)
{
    ros::init(argc,argv,"lfd_recorder_node");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string demo_namebase,planning_group,base_frame, demo_name;
    std::size_t error = 0;
    std::string LOGNAME{"lfd_dmp_baseline"};
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demonstration_name", demo_namebase);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "planning_group", planning_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "base_frame", base_frame);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    MoveitUtil moveit_util(planning_group,base_frame);

    LFDRecorder recorder(moveit_util);

    ROS_INFO("Recording Started, Press s to start a new dmp sequence, Press ctrl+c when recording is finished");

    int demo_count = 0;
    while (ros::ok())
    {
        demo_name = demo_namebase + std::to_string(demo_count);
        recorder.run(demo_name);
        demo_count++;
    }

    ros::waitForShutdown();

}
