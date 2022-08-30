#include <lfd_interface/lfd_controller.h>

LFDController::LFDController(MoveitUtil & moveit_util):
moveit_util_(moveit_util), ac_control_("control_lfd", true)
{

}

LFDController::~LFDController() {}

void LFDController::runControl(lfd_interface::PlanMsg plan_metadata)
{
    
    ac_control_.waitForServer();
    ROS_INFO_NAMED(LOGNAME, "Control action server ready");

    lfd_interface::ControlLFDGoal goal;
    goal.plan = plan_metadata;
    ac_control_.sendGoal(goal);

    bool finished_before_timeout = ac_control_.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_control_.getState();
        ROS_INFO_NAMED(LOGNAME, "Control Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO_NAMED(LOGNAME, "Action did not finish before the time out.");
}