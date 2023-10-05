#include <sstream>

#include <lfd_interface/moveit_util.h>
#include <geometry_msgs/Pose.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

trajectory_msgs::JointTrajectoryPoint getJointState(MoveitUtil & moveit_util, bool print = true)
{
    trajectory_msgs::JointTrajectoryPoint joint_values;
    moveit_util.currentJointState(joint_values);

    if (print)
    {
        ROS_INFO("\n");
        std::stringstream ss;
        ss << "[";
        for (std::size_t i = 0; i < joint_values.positions.size(); ++i)
        {
            ss << joint_values.positions[i];
            if (i != joint_values.positions.size() - 1)  // if not the last joint
            {
                ss << ", ";
            }
        }
        ss << "]";

        ROS_INFO("%s", ss.str().c_str());
    }

    return joint_values;
}

lfd_interface::PoseTrajectoryPoint getPoseState(MoveitUtil & moveit_util, bool print = true)
{
    lfd_interface::PoseTrajectoryPoint pose_values;
    moveit_util.currentPose(pose_values);
    if (print)
    {
        ROS_INFO("\n");
        ROS_INFO("Pose - Position: [x: %f, y: %f, z: %f], Orientation: [x: %f, y: %f, z: %f, w: %f]",
                pose_values.pose.position.x, pose_values.pose.position.y, pose_values.pose.position.z,
                pose_values.pose.orientation.x, pose_values.pose.orientation.y, pose_values.pose.orientation.z, pose_values.pose.orientation.w);

    }

    return pose_values;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"util_node");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string planning_group,base_frame;
    std::string LOGNAME{"util"};
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "planning_group", planning_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "base_frame", base_frame);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    MoveitUtil moveit_util(planning_group,base_frame);

    trajectory_msgs::JointTrajectoryPoint joint_values = getJointState(moveit_util);
    lfd_interface::PoseTrajectoryPoint pose_values = getPoseState(moveit_util);

    auto pose_publisher = nh.advertise<geometry_msgs::Pose>("pose_state", 0);
    ros::Rate loop_rate(50); 

    while (ros::ok())
    {
        lfd_interface::PoseTrajectoryPoint pose_values = getPoseState(moveit_util, false);
        pose_publisher.publish(pose_values.pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    // ros::waitForShutdown();

}
