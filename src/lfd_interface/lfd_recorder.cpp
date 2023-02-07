#include <lfd_interface/lfd_recorder.h>

LFDRecorder::LFDRecorder(MoveitUtil & moveit_util):
moveit_util_(moveit_util)
{

    moveit_util_.publishText("Welcome to LFD Recorder");
    
    pub_save_demonstration_ = nh_.advertise<lfd_interface::DemonstrationMsg>("save_demonstration", 1, false);

    sub_keycommand_ = nh_.subscribe("keyboard_command", 1 ,&LFDRecorder::subCBSetStopFlag,this);
}

LFDRecorder::~LFDRecorder() {}

void LFDRecorder::subCBSetStopFlag(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "s")
    {
        STOP_FLAG = true;
    }
    
}

void LFDRecorder::saveDemonstration()
{
    //publish the trajectory to be saved by the python node
    pub_save_demonstration_.publish(demonstration_);
}

void LFDRecorder::run(std::string demonstration_name)
{
    STOP_FLAG=false;
    demonstration_.joint_trajectory.points.clear();

    trajectory_msgs::JointTrajectoryPoint current_joint_state;
    lfd_interface::PoseTrajectoryPoint current_pose;
    ros::Rate loop_rate(RECORDER_LOOP_RATE);

    demonstration_.joint_trajectory.joint_names = moveit_util_.getMoveGroup()->getJointNames();
    demonstration_.name = demonstration_name;

    auto visual_tools = moveit_util_.getVisualTools();

    ROS_INFO_NAMED(LOGNAME, ("Recording " + demonstration_name + " started, hit ctrl+c when demonstration is finished").c_str());

    //Add current configuration as the starting point
    moveit_util_.currentJointState(current_joint_state);
    moveit_util_.currentPose(current_pose);
    ros::Duration trajpoint_time(0 , 0);
    current_joint_state.time_from_start = trajpoint_time;
    current_pose.time_from_start = trajpoint_time;
    demonstration_.joint_trajectory.points.push_back(current_joint_state);
    demonstration_.pose_trajectory.points.push_back(current_pose);

    //Initialize Rviz visual tools
    visual_tools->deleteAllMarkers();
    moveit_util_.visualizeJointTrajectory(demonstration_.joint_trajectory);

    //Trigger recording when robot starts moving
    while(ros::ok())
    {
        // moveit_util_.currentJointState(current_joint_state);
        moveit_util_.currentPose(current_pose);
        if(robotHasMoved(current_pose)) {
            break;
        }
        loop_rate.sleep();
    }

    //Set Reference time
    ros::Time ref_time = ros::Time::now();
    
    //Main Recording Loop, no extra points will be added if the robot is stationary
    while(!STOP_FLAG && ros::ok())
    {
        moveit_util_.currentPose(current_pose);
        current_pose.time_from_start = ros::Time::now() - ref_time;
        moveit_util_.currentJointState(current_joint_state);
        current_joint_state.time_from_start = ros::Time::now() - ref_time;

        demonstration_.joint_trajectory.points.push_back(current_joint_state);
        demonstration_.pose_trajectory.points.push_back(current_pose);
        saveDemonstration();
        visual_tools->deleteAllMarkers();
        // moveit_util_.visualizeJointTrajectory(demonstration_.joint_trajectory);
        moveit_util_.visualizePosePath(demonstration_.pose_trajectory);

        loop_rate.sleep();
    }
    
}


bool LFDRecorder::robotHasMoved(const lfd_interface::PoseTrajectoryPoint & newstate)
{
    lfd_interface::PoseTrajectoryPoint p1 = demonstration_.pose_trajectory.points.back();
    lfd_interface::PoseTrajectoryPoint p2 = newstate;
    double orientation_diff = orientationDifference(p1.pose.orientation,p2.pose.orientation);
    double position_diff = positionDifference(p1.pose,p2.pose);
    // ROS_INFO_NAMED(LOGNAME, "orientation difference = %.2f%% and position difference =  %.2f%%", orientation_diff, position_diff);
    
    //position threshold is in meters, while orientation threshold is in radians!
    if (position_diff >= 0.01 || orientation_diff >= 0.1)
        return true;
    
    return false;
}

double LFDRecorder::orientationDifference(const geometry_msgs::Quaternion &q1, const geometry_msgs::Quaternion &q2)
{
  double dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
  //one orientation can be represented with two quaternions, hence we might get pi difference instead of zero
  //So here we make sure the dot product is positive so that we always get the same answer!
  if (dot <0)
    dot = -dot;

  double norm1 = sqrt(q1.x * q1.x + q1.y * q1.y + q1.z * q1.z + q1.w * q1.w);
  double norm2 = sqrt(q2.x * q2.x + q2.y * q2.y + q2.z * q2.z + q2.w * q2.w);
  double cos_theta = dot / (norm1 * norm2);

  double orientation_difference = 2 * acos(cos_theta);

  return orientation_difference;
}

double LFDRecorder::positionDifference(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
{
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  double dz = p1.position.z - p2.position.z;

  double position_distance = sqrt(dx * dx + dy * dy + dz * dz);

  return position_distance;
}