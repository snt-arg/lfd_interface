#include <lfd_interface/lfd_recorder.h>

LFDRecorder::LFDRecorder(std::string demonstration_name, std::string planning_group, std::string base_frame,ros::NodeHandle& pnh):
demonstration_name_(demonstration_name),pnh_(pnh),
move_group_interface_(planning_group), visual_tools_(base_frame)
{
    joint_model_group_ = move_group_interface_.getCurrentState()->getJointModelGroup(planning_group);

    visual_tools_.deleteAllMarkers();
    visual_tools_.loadRemoteControl();

    publishText("Welcome to LFD Recorder");

    ROS_INFO_NAMED(LOGNAME, "Reference Frame: %s", move_group_interface_.getPlanningFrame().c_str());
    ROS_INFO_NAMED(LOGNAME, "End-Effector Frame: %s", move_group_interface_.getEndEffectorLink().c_str());
    ROS_INFO_NAMED(LOGNAME, "Available Planning Groups:");
    std::copy(move_group_interface_.getJointModelGroupNames().begin(),
        move_group_interface_.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    pub_save_trajectory_ = nh_.advertise<lfd_interface::Demonstration>("save_demonstration", 1, false);
}

LFDRecorder::~LFDRecorder() {}

void LFDRecorder::publishText(std::string text)
{
    namespace rvt = rviz_visual_tools;
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools_.publishText(text_pose, text.c_str(), rvt::WHITE, rvt::XLARGE);
    visual_tools_.trigger();

}

trajectory_msgs::JointTrajectoryPoint LFDRecorder::currentJointState()
{
    moveit::core::RobotStatePtr current_state = move_group_interface_.getCurrentState();
    trajectory_msgs::JointTrajectoryPoint joint_values;

    current_state->copyJointGroupPositions(joint_model_group_, joint_values.positions);
    return joint_values;
}

void LFDRecorder::visualizeTrajectory()
{
    moveit_msgs::RobotTrajectory viz_trajectory;
    viz_trajectory.joint_trajectory = joint_trajectory_;
    
    visual_tools_.publishTrajectoryLine(viz_trajectory, joint_model_group_);
    visual_tools_.trigger();
}

void LFDRecorder::saveDemonstration()
{
    lfd_interface::Demonstration demonstration;
    demonstration.name = demonstration_name_;
    demonstration.joint_trajectory = joint_trajectory_;

    //publish the trajectory to be saved by the python node
    pub_save_trajectory_.publish(demonstration);
}

void LFDRecorder::run()
{
    int i = 0;
    std::string prompt;

    joint_trajectory_.joint_names = move_group_interface_.getJointNames();

    ROS_INFO_NAMED(LOGNAME, "Recording started, hit ctrl+c when demonstration is finished");

    while(ros::ok())
    {
        i++;
        prompt = "Ready to record Configuration #" + std::to_string(i) + ".Hit Next when in desired configuration";
        visual_tools_.prompt(prompt);
        visual_tools_.deleteAllMarkers();
        publishText("Configuration #" + std::to_string(i));
        joint_trajectory_.points.push_back(currentJointState());
        visualizeTrajectory();
        saveDemonstration();
    }
    
}

