#include <lfd_interface/lfd_planner.h>

LFDPlanner::LFDPlanner(std::string demonstration_name, MoveitUtil & moveit_util):
demonstration_name_(demonstration_name), trainer(demonstration_name),
moveit_util_(moveit_util)
{
    demonstration_ = trainer.fetchDemonstration();

    client_plan_lfd_ = nh_.serviceClient<lfd_interface::PlanLFD>("plan_lfd");

    auto visual_tools = moveit_util_.getVisualTools();
    visual_tools->deleteAllMarkers();

    pub_displayplan_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 10);
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
    moveit_util_.getVisualTools()->prompt("press next to get the plan and visualize the planned trajectory");
    getPlan(demonstration_.joint_trajectory.points.front(),
            demonstration_.joint_trajectory.points.back());
    
    moveit_util_.visualizeJointTrajectory(plan_);
    refine();
    displayPlannedPath();

    moveit_util_.getVisualTools()->prompt("press next to execute the planned trajectory");
    
    executePlan();
}

void LFDPlanner::refine()
{
    trajectory_msgs::JointTrajectoryPoint jtp = demonstration_.joint_trajectory.points.back();
    std::vector<double> zero_vel(jtp.positions.size(),0);
    jtp.velocities = zero_vel;

    jtp.time_from_start = jtp.time_from_start + ros::Duration(0.5);

    plan_.points.push_back(jtp);
}

void LFDPlanner::displayPlannedPath()
{
    moveit_msgs::DisplayTrajectory dt;
    moveit_msgs::RobotTrajectory rt;
    
    
    dt.model_id = "yumi";
    rt.joint_trajectory = plan_;
    dt.trajectory.push_back(rt);
    pub_displayplan_.publish(dt);
}

void LFDPlanner::executePlan()
{
    moveit_msgs::RobotTrajectory rt;
    rt.joint_trajectory = plan_;
    moveit_util_.getMoveGroup()->execute(rt);
}