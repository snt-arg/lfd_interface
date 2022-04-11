#include <lfd_interface/moveit_util.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <lfd_interface/GetDemonstration.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"test_node");
    ros::AsyncSpinner spinner(1);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    spinner.start();

    std::string demonstration_name, planning_group, base_frame;
    std::size_t error = 0;
    std::string LOGNAME{"test_node"};
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demonstration_name", demonstration_name);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "planning_group", planning_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "base_frame", base_frame);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);
    
    MoveitUtil moveit_util(planning_group,base_frame);
    ros::ServiceClient sc;
    sc = nh.serviceClient<lfd_interface::GetDemonstration>("get_demonstration");

    sc.waitForExistence();

    lfd_interface::GetDemonstration srv;
    srv.request.name = demonstration_name;

    if(sc.call(srv))
    {
        moveit_util.visualizeJointTrajectory(srv.response.Demonstration.joint_trajectory);
    }

    ros::waitForShutdown();

}
