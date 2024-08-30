#include <lfd_interface/lfd_planner.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lfd_planner_node");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string demonstration_name,planning_group,base_frame, robot_ns;
    std::size_t error = 0;
    std::string LOGNAME{"lfd_planner"};
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demonstration_name", demonstration_name);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "planning_group", planning_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "base_frame", base_frame);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "robot_ns", robot_ns);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    MoveitUtil moveit_util(planning_group,base_frame);

    LFDPlanner planner(moveit_util,robot_ns);
    planner.init(demonstration_name);
    planner.runViz(); 

    ros::waitForShutdown();

}
