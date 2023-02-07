#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


#include <lfd_interface/PoseTrajectory.h>
#include <lfd_interface/PoseTrajectoryPoint.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "fr3_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("fr3_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -tau / 6;  // -1/6 turn in radians
  move_group_interface.setJointValueTarget(joint_group_positions);

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group_interface.setMaxVelocityScalingFactor(0.05);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  ////////////////////////////////////


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  // const moveit::core::JointModelGroup* joint_model_group2 = kinematic_model->getJointModelGroup("fr3_arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retrieve the current set of joint values stored in the state for the Panda arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  lfd_interface::PoseTrajectory pose_trajectory;
  lfd_interface::PoseTrajectoryPoint trajectory_point;
  std::vector<geometry_msgs::Pose> waypoints;

  for (trajectory_msgs::JointTrajectoryPoint point : my_plan.trajectory_.joint_trajectory.points)
  {
    kinematic_state->setJointGroupPositions(joint_model_group, point.positions);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("fr3_link8");
    trajectory_point.pose = tf2::toMsg(end_effector_state);
    trajectory_point.time_from_start = point.time_from_start;
    pose_trajectory.points.push_back(trajectory_point);
    waypoints.push_back(trajectory_point.pose);
    visual_tools.publishAxisLabeled(end_effector_state, "s");
    visual_tools.trigger();
  }

  // moveit::planning_interface::MoveGroupInterface::Plan rt_plan;
  moveit_msgs::RobotTrajectory rt;

  double fraction = move_group_interface.computeCartesianPath(waypoints,0.01,0.0,rt);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
  
  visual_tools.deleteAllMarkers();
  visual_tools.publishPath(waypoints);
  // visual_tools.publishTrajectoryLine(rt,joint_model_group2);
  visual_tools.trigger();


  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  // visual_tools.publishAxisLabeled(end_effector_state, "s");

  // trajectory_msgs::JointTrajectoryPoint point = my_plan.trajectory_.joint_trajectory.points[5];
  // kinematic_state->setJointGroupPositions(joint_model_group2, point.positions);
  // const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("fr3_link8");

  // // auto d = tf2::eigenToTransform(end_effector_state);
  // auto d = tf2::toMsg(end_effector_state);

  // // Visualize the plan in RViz
  // // visual_tools.deleteAllMarkers();
  


  // // Joint Limits
  // // ^^^^^^^^^^^^
  // // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  // /* Set one joint in the Panda arm outside its joint limit */
  // joint_values[0] = 5.57;
  // kinematic_state->setJointGroupPositions(joint_model_group2, joint_values);

  // /* Check whether any joint is outside its joint limits */
  // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // /* Enforce the joint limits for this state and check again*/
  // kinematic_state->enforceBounds();
  // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // // Forward Kinematics
  // // ^^^^^^^^^^^^^^^^^^
  // // Now, we can compute forward kinematics for a set of random joint
  // // values. Note that we would like to find the pose of the
  // // "panda_link8" which is the most distal link in the
  // // "panda_arm" group of the robot.
  // kinematic_state->setToRandomPositions(joint_model_group2);
  // const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("fr3_link8");

  // /* Print end-effector pose. Remember that this is in the model frame */
  // ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  // ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  // // Inverse Kinematics
  // // ^^^^^^^^^^^^^^^^^^
  // // We can now solve inverse kinematics (IK) for the Panda robot.
  // // To solve IK, we will need the following:
  // //
  // //  * The desired pose of the end-effector (by default, this is the last link in the "panda_arm" chain):
  // //    end_effector_state that we computed in the step above.
  // //  * The timeout: 0.1 s
  // double timeout = 0.1;
  // bool found_ik = kinematic_state->setFromIK(joint_model_group2, end_effector_state, timeout);

  // // Now, we can print out the IK solution (if found):
  // if (found_ik)
  // {
  //   kinematic_state->copyJointGroupPositions(joint_model_group2, joint_values);
  //   for (std::size_t i = 0; i < joint_names.size(); ++i)
  //   {
  //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  //   }
  // }
  // else
  // {
  //   ROS_INFO("Did not find IK solution");
  // }

  // // Get the Jacobian
  // // ^^^^^^^^^^^^^^^^
  // // We can also get the Jacobian from the :moveit_core:`RobotState`.
  // Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  // Eigen::MatrixXd jacobian;
  // kinematic_state->getJacobian(joint_model_group2,
  //                              kinematic_state->getLinkModel(joint_model_group2->getLinkModelNames().back()),
  //                              reference_point_position, jacobian);
  // ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  // // END_TUTORIAL

  ros::shutdown();
  return 0;
}