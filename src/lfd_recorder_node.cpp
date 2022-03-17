#include <lfd_interface/lfd_recorder.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>


int main(int argc, char** argv)
{
    ros::init(argc,argv,"lfd_recorder_node");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string demonstration_name,planning_group,base_frame;
    std::size_t error = 0;
    std::string LOGNAME{"lfd_recorder"};
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demonstration_name", demonstration_name);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "planning_group", planning_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "base_frame", base_frame);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    MoveitUtil moveit_util(planning_group,base_frame);

    LFDRecorder recorder(moveit_util);
    recorder.run(demonstration_name);

    ros::waitForShutdown();

}
