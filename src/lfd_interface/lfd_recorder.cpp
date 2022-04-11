#include <lfd_interface/lfd_recorder.h>

LFDRecorder::LFDRecorder(MoveitUtil & moveit_util):
moveit_util_(moveit_util)
{

    moveit_util_.publishText("Welcome to LFD Recorder");
    
    pub_save_demonstration_ = nh_.advertise<lfd_interface::DemonstrationMsg>("save_demonstration", 1, false);
}

LFDRecorder::~LFDRecorder() {}


void LFDRecorder::saveDemonstration()
{
    //publish the trajectory to be saved by the python node
    pub_save_demonstration_.publish(demonstration_);
}

void LFDRecorder::run(std::string demonstration_name)
{
    trajectory_msgs::JointTrajectoryPoint current_joint_state;
    ros::Rate loop_rate(RECORDER_LOOP_RATE);

    demonstration_.joint_trajectory.joint_names = moveit_util_.getMoveGroup()->getJointNames();
    demonstration_.name = demonstration_name;

    auto visual_tools = moveit_util_.getVisualTools();

    ROS_INFO_NAMED(LOGNAME, "Recording started, hit ctrl+c when demonstration is finished");

    //Add current configuration as the starting point
    moveit_util_.currentJointState(current_joint_state);
    ros::Duration trajpoint_time(0 , 0);
    current_joint_state.time_from_start = trajpoint_time;
    demonstration_.joint_trajectory.points.push_back(current_joint_state);

    //Initialize Rviz visual tools
    visual_tools->deleteAllMarkers();
    moveit_util_.visualizeJointTrajectory(demonstration_.joint_trajectory);

    //Trigger recording when robot starts moving
    while(true)
    {
        moveit_util_.currentJointState(current_joint_state);
        if(robotHasMoved(current_joint_state)) {
            break;
        }
        loop_rate.sleep();
    }

    //Set Reference time
    ros::Time ref_time = ros::Time::now();
    
    //Main Recording Loop, no extra points will be added if the robot is stationary
    while(ros::ok())
    {
        moveit_util_.currentJointState(current_joint_state);
        current_joint_state.time_from_start = ros::Time::now() - ref_time;

        if(robotHasMoved(current_joint_state))
        {
            demonstration_.joint_trajectory.points.push_back(current_joint_state);
            saveDemonstration();
            visual_tools->deleteAllMarkers();
            moveit_util_.visualizeJointTrajectory(demonstration_.joint_trajectory);
        }

        loop_rate.sleep();
    }
    
}

bool LFDRecorder::robotHasMoved(const trajectory_msgs::JointTrajectoryPoint & newstate)
{
    for (size_t i = 0; i < newstate.positions.size(); i++)
    {
        double diff = fabs(demonstration_.joint_trajectory.points.back().positions[i] - newstate.positions[i]);
        if (diff > JOINTMOVE_THRESH)
            return true;
    }
    return false;
    
}

