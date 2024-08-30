#include <sstream>

#include <lfd_interface/moveit_util.h>
#include <geometry_msgs/Pose.h>

#include <actionlib/server/simple_action_server.h>
#include <lfd_interface/PlanJointAction.h>

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

class PlanJointAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<lfd_interface::PlanJointAction> as_;
    std::string action_name_;

    lfd_interface::PlanJointResult result_;

    MoveitUtil& moveit_util_;

public:
    PlanJointAction(std::string name, MoveitUtil & moveit_util) :
        as_(nh_, name, boost::bind(&PlanJointAction::executeCB, this, _1), false),
        action_name_(name), moveit_util_(moveit_util)
    {
        as_.start();
    }

    ~PlanJointAction(void) {}

    void executeCB(const lfd_interface::PlanJointGoalConstPtr &goal)
    {
        moveit_util_.planPath(goal->joint_position.positions);
        moveit_util_.getVisualTools()->prompt("press next to execute the planned trajectory");
        moveit_util_.move();

        result_.success = true;
        as_.setSucceeded(result_);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"util_node");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string planning_group,base_frame, robot_ns;
    std::string LOGNAME{"util"};
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "planning_group", planning_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "base_frame", base_frame);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "robot_ns", robot_ns);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    MoveitUtil moveit_util(planning_group,base_frame);

    trajectory_msgs::JointTrajectoryPoint joint_values = getJointState(moveit_util);
    lfd_interface::PoseTrajectoryPoint pose_values = getPoseState(moveit_util);

    auto pose_publisher = nh.advertise<geometry_msgs::Pose>(robot_ns + "/pose_state", 0);
    ros::Rate loop_rate(50); 

    // std::vector<double> joint_positions = {-0.00533887, -0.80099008, 0.00345199, -2.37769808, 0.00247941, 1.57670778, 0.78244555};
    // moveit_util.planPath(joint_positions);
    // moveit_util.move();

    PlanJointAction as_plan_joint(robot_ns + "/plan_joint", moveit_util);

    while (ros::ok())
    {
        lfd_interface::PoseTrajectoryPoint pose_values = getPoseState(moveit_util, false);
        pose_publisher.publish(pose_values.pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    // ros::waitForShutdown();

}
