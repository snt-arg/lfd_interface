#include <lfd_interface/moveit_util.h>

//TODO: viz_base_frame is no longer used, remove it from the whole code!
MoveitUtil::MoveitUtil(std::string planning_group, std::string viz_base_frame):
robot_model_loader_("robot_description"), kinematic_model_(robot_model_loader_.getModel()),
kinematic_state_(new moveit::core::RobotState(kinematic_model_))
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group);
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(move_group_->getPlanningFrame());

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

lfd_interface::PoseTrajectoryPoint MoveitUtil::jointToPose(trajectory_msgs::JointTrajectoryPoint & joint_states)
{
    kinematic_state_->setJointGroupPositions(joint_model_group_,joint_states.positions);
    const Eigen::Isometry3d& end_effector_state = kinematic_state_->getGlobalLinkTransform(move_group_->getEndEffectorLink());
    lfd_interface::PoseTrajectoryPoint pose_values;
    pose_values.pose = tf2::toMsg(end_effector_state);
    return pose_values;
}

void MoveitUtil::currentPose(lfd_interface::PoseTrajectoryPoint & pose_values)
{
    trajectory_msgs::JointTrajectoryPoint joint_states;
    currentJointState(joint_states);
    pose_values = jointToPose(joint_states);
    // visual_tools_->publishAxisLabeled(pose, "s");
    // visual_tools_->trigger();
}

void MoveitUtil::visualizePosePath(lfd_interface::PoseTrajectory & trajectory)
{
    std::vector<geometry_msgs::Pose> waypoints;
    for(lfd_interface::PoseTrajectoryPoint point: trajectory.points)
    {
        waypoints.push_back(point.pose);
    }
    
    visual_tools_->publishPath(waypoints, active_color_);
    visual_tools_->trigger();
}