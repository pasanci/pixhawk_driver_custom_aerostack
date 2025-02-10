#include "pixhawk_command_interface.hpp"
#include "ros_utils_lib/ros_utils.hpp"

int main(int argc,char **argv)
{
    //Ros Init
    
    ros::init(argc, argv, ros_utils_lib::getNodeName("Pixhawk_command_interface"));

    std::cout<<"[ROSNODE] Starting Pixhawk Command Interface"<<std::endl;

    //Vars
    CommandInterface CommandInterface;
    CommandInterface.setUp();
    
    try{
        CommandInterface.start();
    }
    catch (std::exception &ex){
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
        CommandInterface.stop();
    }
    
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        CommandInterface.run();
        loop_rate.sleep();
    }
    return 0;
}
