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
    int i = 0;
    std::string prompt;

    demonstration_.joint_trajectory.joint_names = moveit_util_.getMoveGroup()->getJointNames();
    demonstration_.name = demonstration_name;

    auto visual_tools = moveit_util_.getVisualTools();

    ROS_INFO_NAMED(LOGNAME, "Recording started, hit ctrl+c when demonstration is finished");

    while(ros::ok())
    {
        i++;
        prompt = "Ready to record Configuration #" + std::to_string(i) + ".Hit Next when in desired configuration";
        visual_tools->prompt(prompt);
        visual_tools->deleteAllMarkers();
        moveit_util_.publishText("Configuration #" + std::to_string(i));
        demonstration_.joint_trajectory.points.push_back(moveit_util_.currentJointState());
        moveit_util_.visualizeJointTrajectory(demonstration_.joint_trajectory);
        saveDemonstration();
    }
    
}

