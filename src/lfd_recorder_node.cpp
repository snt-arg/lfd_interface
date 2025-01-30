#include <lfd_interface/lfd_recorder.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>


int main(int argc, char** argv)
{
    ros::init(argc,argv,"lfd_recorder_node");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string demo_namebase,planning_group,base_frame, demo_name, robot_name, description;
    std::size_t error = 0;
    std::string LOGNAME{"lfd_dmp_baseline"};
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demonstration_name", demo_namebase);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "planning_group", planning_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "base_frame", base_frame);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "robot_name", robot_name);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "description", description);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    MoveitUtil moveit_util(planning_group,base_frame);

    LFDRecorder recorder(moveit_util);

    ROS_INFO("Recording Started, Press s to finish recording");

    demo_name = demo_namebase;
    recorder.run(demo_name, robot_name, description);
}
