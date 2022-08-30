#include <lfd_interface/lfd_controller.h>

LFDController::LFDController(MoveitUtil & moveit_util):
moveit_util_(moveit_util), ac_control_("control_lfd", true), loop_rate(20)
{
    recorded_traj_.joint_names = moveit_util_.getMoveGroup()->getJointNames();
    
}

LFDController::~LFDController() {}

void LFDController::runControl(lfd_interface::PlanMsg plan_metadata)
{
    
    ac_control_.waitForServer();
    ROS_INFO_NAMED(LOGNAME, "Control action server ready");
    lfd_interface::ControlLFDGoal goal;
    goal.plan = plan_metadata;
    ac_control_.sendGoal(goal);

    recorded_traj_.points.clear();
    trajectory_msgs::JointTrajectoryPoint current_state;

    while(ros::ok())
    {
        loop_rate.sleep();
        actionlib::SimpleClientGoalState state = ac_control_.getState();

        if (state == actionlib::SimpleClientGoalState::ACTIVE ||
            state == actionlib::SimpleClientGoalState::PENDING)
        {
            moveit_util_.currentJointState(current_state);
            recorded_traj_.points.push_back(current_state);
        }
        else break;
    }

    ROS_INFO("Showing recorded trajectory in Red");
    moveit_util_.changeVisualizationColor(rviz_visual_tools::RED);
    moveit_util_.visualizeJointTrajectory(recorded_traj_);


    actionlib::SimpleClientGoalState state = ac_control_.getState();
    ROS_INFO_NAMED(LOGNAME, "Control Action finished: %s",state.toString().c_str());

}
