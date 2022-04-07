#include <lfd_interface/lfd_planner.h>

LFDPlanner::LFDPlanner(std::string demonstration_name, MoveitUtil & moveit_util):
demonstration_name_(demonstration_name), trainer(demonstration_name),
moveit_util_(moveit_util)
{
    demonstration_ = trainer.fetchDemonstration();

    client_plan_lfd_ = nh_.serviceClient<lfd_interface::PlanLFD>("plan_lfd");

    auto visual_tools = moveit_util_.getVisualTools();
    visual_tools->deleteAllMarkers();
}

LFDPlanner::~LFDPlanner()
{}

void LFDPlanner::getPlan(trajectory_msgs::JointTrajectoryPoint start,
                trajectory_msgs::JointTrajectoryPoint goal)
{
    client_plan_lfd_.waitForExistence();
    lfd_interface::PlanLFD srv;
    srv.request.name = demonstration_name_;
    srv.request.start = start;
    srv.request.goal = goal;

    if(client_plan_lfd_.call(srv))
    {
        plan_ = srv.response.trajectory;
    }
}

void LFDPlanner::run()
{
    moveit_util_.visualizeJointTrajectory(demonstration_.joint_trajectory);
    auto demo_size = demonstration_.joint_trajectory.points.size();
    getPlan(demonstration_.joint_trajectory.points[0],
            demonstration_.joint_trajectory.points[demo_size-1]);
    
    moveit_util_.visualizeJointTrajectory(plan_);
}