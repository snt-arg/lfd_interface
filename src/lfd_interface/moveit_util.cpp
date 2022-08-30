#include <lfd_interface/moveit_util.h>

MoveitUtil::MoveitUtil(std::string planning_group, std::string viz_base_frame)
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group);
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(viz_base_frame);

    joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(planning_group);

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    ROS_INFO_NAMED(LOGNAME, "Reference Frame: %s", move_group_->getPlanningFrame().c_str());
    ROS_INFO_NAMED(LOGNAME, "End-Effector Frame: %s", move_group_->getEndEffectorLink().c_str());
    ROS_INFO_NAMED(LOGNAME, "Available Planning Groups:");
    std::copy(move_group_->getJointModelGroupNames().begin(),
        move_group_->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

MoveitUtil::~MoveitUtil()
{}

MoveitUtil::MoveGroupPtr MoveitUtil::getMoveGroup()
{
    return move_group_;
}

MoveitUtil::VisualToolsPtr MoveitUtil::getVisualTools()
{
    return visual_tools_;
}

void MoveitUtil::publishText(std::string text)
{
    namespace rvt = rviz_visual_tools;
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools_->publishText(text_pose, text.c_str(), active_color_, rvt::XLARGE);
    visual_tools_->trigger();

}

void MoveitUtil::currentJointState(trajectory_msgs::JointTrajectoryPoint & joint_values)
{
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

    current_state->copyJointGroupPositions(joint_model_group_, joint_values.positions);
}

void MoveitUtil::visualizeJointTrajectory(trajectory_msgs::JointTrajectory & trajectory)
{
    moveit_msgs::RobotTrajectory viz_trajectory;
    viz_trajectory.joint_trajectory = trajectory;
    
    visual_tools_->publishTrajectoryLine(viz_trajectory, joint_model_group_, active_color_);
    visual_tools_->trigger();
}

void MoveitUtil::changeVisualizationColor(enum rviz_visual_tools::colors color)
{
     active_color_ = color;
}