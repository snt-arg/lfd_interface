#include <lfd_interface/lfd_trainer.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lfd_trainer_node");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Fetch params
    std::string demonstration_name;
    std::size_t error = 0;
    std::string LOGNAME{"lfd_trainer"};
    error += !rosparam_shortcuts::get(LOGNAME, pnh, "demonstration_name", demonstration_name);
    rosparam_shortcuts::shutdownIfError(LOGNAME, error);

    LFDTrainer trainer(demonstration_name);
    trainer.run(); 

    ros::waitForShutdown();

}
