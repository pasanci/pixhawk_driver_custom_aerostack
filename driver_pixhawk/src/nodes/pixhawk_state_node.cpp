#include "pixhawk_state_interface.hpp"
#include "ros_utils_lib/ros_utils.hpp"

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv,ros_utils_lib::getNodeName("Pixhawk_state_interface"));

    std::cout<<"[ROSNODE] Starting Pixhawk State Interface"<<std::endl;

    //Vars
    StateInterface stateInterface;
    stateInterface.setUp();
    
    try{
        stateInterface.start();
    }
    catch (std::exception &ex){
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
        stateInterface.stop();
    }

    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        ros::spinOnce();
        stateInterface.run();
        loop_rate.sleep();
    }
    return 0;
}
