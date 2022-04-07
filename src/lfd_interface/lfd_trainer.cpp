#include <lfd_interface/lfd_trainer.h>

LFDTrainer::LFDTrainer(std::string demonstration_name):
demonstration_name_(demonstration_name)
{
    client_load_demonstration_ = nh_.serviceClient<lfd_interface::GetDemonstration>("get_demonstration");

    client_train_demonstration_ = nh_.serviceClient<lfd_interface::TrainDemonstration>("train_demonstration");

}

LFDTrainer::~LFDTrainer() {}

bool LFDTrainer::loadDemonstration()
{
    client_load_demonstration_.waitForExistence();
    srv_get_demonstration_.request.name = demonstration_name_;

    if(client_load_demonstration_.call(srv_get_demonstration_)
       && srv_get_demonstration_.response.success)
    {
        demonstration_ = srv_get_demonstration_.response.Demonstration;
        return true;
    }
    else
    {
        ROS_ERROR_NAMED(LOGNAME, "Failed to load demonstration file");
        return false;
    }
}

lfd_interface::DemonstrationMsg LFDTrainer::fetchDemonstration()
{
    loadDemonstration();
    return demonstration_;
}

bool LFDTrainer::trainDemonstration()
{
    client_train_demonstration_.waitForExistence();
    srv_train_demonstration_.request.demonstration = demonstration_;

    if(client_train_demonstration_.call(srv_train_demonstration_))
    {
        return srv_train_demonstration_.response.success;
    }
    else
    {
        ROS_ERROR_NAMED(LOGNAME, "Failed to call train demonstration service");
        return false;
    }
}

bool LFDTrainer::run()
{
    if(loadDemonstration() && trainDemonstration())
    {
        return true;
    }
    return false;
}

