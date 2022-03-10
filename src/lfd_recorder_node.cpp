#include <lfd_interface/lfd_recorder.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lfd_interface_node");
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

    LFDRecorder recorder(demonstration_name,planning_group,base_frame,pnh);
    recorder.run();

    ros::waitForShutdown();

}
