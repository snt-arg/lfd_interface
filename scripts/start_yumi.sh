#!/bin/bash

rosservice call /yumi/rws/pp_to_main "{}"
rosservice call /yumi/rws/start_rapid "{}"
roslaunch yumi_bringup set_egm_settings.launch
rosservice call /yumi/rws/sm_addin/get_egm_settings "task: 'T_ROB_L'"