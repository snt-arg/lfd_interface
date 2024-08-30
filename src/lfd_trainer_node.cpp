#include <lfd_interface/lfd_trainer.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lfd_trainer_node");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string demonstration_name,robot_ns;
    std::size_t error = 0;
    std::string LOGNAME{"lfd_trainer"};
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demonstration_name", demonstration_name);
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "robot_ns", robot_ns);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    LFDTrainer trainer(robot_ns);
    trainer.init(demonstration_name);
    trainer.run(); 

    ros::waitForShutdown();

}
