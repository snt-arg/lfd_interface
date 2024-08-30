#include <lfd_interface/lfd_planner.h>

LFDPlanner::LFDPlanner(MoveitUtil & moveit_util, std::string robot_ns):
moveit_util_(moveit_util), trainer(robot_ns)
{
    client_plan_lfd_ = nh_.serviceClient<lfd_interface::PlanLFD>(robot_ns + "/plan_lfd");

    auto visual_tools = moveit_util_.getVisualTools();
    visual_tools->deleteAllMarkers();

    pub_displayplan_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 10);
}

LFDPlanner::~LFDPlanner()
{}

void LFDPlanner::init(std::string demonstration_name,
                    trajectory_msgs::JointTrajectoryPoint goal_joint,
                    double duration/*=0.0*/)
{
    demonstration_name_ = demonstration_name;
    duration_ = duration;
    trainer.init(demonstration_name);
    demonstration_ = trainer.fetchDemonstration();
    if (goal_joint != trajectory_msgs::JointTrajectoryPoint()) 
        goal_joint_ = goal_joint;
    else
        goal_joint_ = demonstration_.joint_trajectory.points.back();
}

void LFDPlanner::getPlan(trajectory_msgs::JointTrajectoryPoint start,
                trajectory_msgs::JointTrajectoryPoint goal)
{
    client_plan_lfd_.waitForExistence();
    plan_metadata_.name = demonstration_name_;
    plan_metadata_.start = start;
    plan_metadata_.goal = goal;
    if (duration_ == 0.0)
        plan_metadata_.tau = 1;
    else
        plan_metadata_.tau = duration_;

    lfd_interface::PlanLFD srv;
    srv.request.plan = plan_metadata_;

    if(client_plan_lfd_.call(srv))
    {
        plan_ = srv.response.trajectory;
    }
}

void LFDPlanner::visualizePlannedTrajectory()
{
    
    moveit_util_.changeVisualizationColor(rviz_visual_tools::LIME_GREEN);
    moveit_util_.visualizeJointTrajectory(demonstration_.joint_trajectory);

    moveit_util_.getVisualTools()->prompt("press next to get the plan and visualize the planned trajectory");

    trajectory_msgs::JointTrajectoryPoint start;
    moveit_util_.currentJointState(start);

    getPlan(start , goal_joint_);
    
    // refine();
    moveit_util_.changeVisualizationColor(rviz_visual_tools::LIME_GREEN);
    moveit_util_.visualizeJointTrajectory(plan_);
    displayPlannedPath();

}

lfd_interface::PlanMsg LFDPlanner::fetchPlanMetaData()
{
    return plan_metadata_;
}

trajectory_msgs::JointTrajectory LFDPlanner::runViz()
{
    visualizePlannedTrajectory();
    return plan_;
    // moveit_util_.getVisualTools()->prompt("press next to execute the planned trajectory");
    
    // executePlan();
}

void LFDPlanner::runExec()
{
    trajectory_msgs::JointTrajectoryPoint start;
    moveit_util_.currentJointState(start);

    getPlan(start , goal_joint_);
    
    // refine();

    executePlan();

}

void LFDPlanner::refine()
{
    trajectory_msgs::JointTrajectoryPoint jtp = goal_joint_;
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