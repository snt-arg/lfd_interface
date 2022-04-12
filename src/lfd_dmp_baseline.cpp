#include <lfd_interface/lfd_recorder.h>
#include <lfd_interface/lfd_trainer.h>
#include <lfd_interface/lfd_planner.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

std::string key;

void subCBKeyCommand(const std_msgs::String::ConstPtr& msg)
{
    key = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lfd_dmp_baseline");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string demo_namebase,planning_group,base_frame, demo_name;
    std::size_t error = 0, demo_count;
    std::string LOGNAME{"lfd_dmp_baseline"};
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demonstration_name", demo_namebase);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "planning_group", planning_group);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "base_frame", base_frame);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demo_count", demo_count);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);


    ros::Subscriber sub_keycommand = nh.subscribe("keyboard_command", 1 ,subCBKeyCommand);

    MoveitUtil moveit_util(planning_group,base_frame);

    LFDTrainer lfd_trainer;
    LFDPlanner lfd_planner(moveit_util);


    ROS_INFO("Press t to start training");

    while (ros::ok() && key != "t");

    ROS_INFO("Training Started");

    for (size_t i = 0; i < demo_count; i++)
    {
        lfd_trainer.init(demo_namebase + std::to_string(i));
        lfd_trainer.run();
    }

    ROS_INFO("Training Finished, Press p to start planning with visualization, press e to directly execute the sequence");

    while (ros::ok())
    {
        if (key == "p")
        {
            for (size_t i = 0; i < demo_count; i++)
            {
                lfd_planner.init(demo_namebase + std::to_string(i));
                lfd_planner.runViz();
            }
            break;
        }
        else if (key == "e")
        {
            for (size_t i = 0; i < demo_count; i++)
            {
                lfd_planner.init(demo_namebase + std::to_string(i));
                lfd_planner.runExec();
            }
            break;
        }
        
    }
    

    ros::waitForShutdown();

}
