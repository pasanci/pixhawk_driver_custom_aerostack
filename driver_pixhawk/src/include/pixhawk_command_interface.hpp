#pragma once

#include "ros/ros.h"
#include "aerostack_msgs/FlightActionCommand.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/Thrust.h"


#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include "mav_msgs/RollPitchYawrateThrust.h"

#include "tf/transform_datatypes.h"


#include <robot_process.h>
#include "ros_utils_lib/ros_utils.hpp"


#include <iostream>

#define USE_DEPRECATED_CONTROL_TOPICS 0

class CommandInterface : public RobotProcess{
private:
    ros::NodeHandle nh_;

    std::string n_space_;
    bool simulation_mode_ = true;
    bool acro_mode_ = false;
    
    // subscribers
    std::string aerostack_flight_action_command_topic_;
    std::string mavros_state_topic_;
    std::string mavros_imu_topic_;
    std::string aerostack_roll_pitch_yaw_rate_thrust_command_topic_;
    std::string aerostack_speed_command_topic_;
    std::string aerostack_pose_command_topic_;
    std::string aerostack_thottle_command_topic_;
    
    // publishers
    std::string mavros_attitude_command_topic_;
    std::string mavros_cmd_vel_command_topic_;
    std::string mavros_thrust_command_topic_;

    // services
    std::string mavros_arming_service_;
    std::string mavros_takeoff_service_;
    std::string mavros_landing_service_;
    std::string mavros_set_mode_service_;

    
    

    // Flight Action Commands
    ros::Subscriber flight_commands_sub_;

    ros::Subscriber state_sub_;

    ros::Subscriber throttle_sub_; //TEST
    void thrustCallback(const mavros_msgs::Thrust&);
    mavros_msgs::Thrust throttle_msg_;


    ros::Subscriber deprecated_attitude_thurst_sub_;
    ros::Subscriber datittude_sub_;
    ros::Subscriber atittude_sub_;
    void alttitudeThrustCallback(const mav_msgs::RollPitchYawrateThrust&);
    void dAttitudeCallback(const geometry_msgs::TwistStamped &);
    void attitudeCallback(const geometry_msgs::PoseStamped &);

    ros::Subscriber imu_sub_; 
    bool initial_yaw_fixed_ = false;
    double initial_yaw = 0.0f;
    double dyaw = 0.0f;
    void CallbackImuTopic(const sensor_msgs::Imu& );
    

    ros::ServiceClient arming_client_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient landing_client_;
    ros::ServiceClient set_mode_client_;
    
    ros::Publisher flight_commands_pub_;
    ros::Publisher attitude_setpoint_pub_;
    ros::Publisher dattitude_setpoint_pub_;
    ros::Publisher thrust_setpoint_pub_;
    ros::Publisher control_mode;
    
    
    void flightActionCallback(const aerostack_msgs::FlightActionCommand& );
    void UAVStateCallback(const mavros_msgs::State& );
    void checkOffboardMode(); 

    bool armVehicle(const bool& );

    mavros_msgs::State current_state_;
    mavros_msgs::Thrust thrust_msg_;
    geometry_msgs::TwistStamped dattitude_msg_;
    geometry_msgs::PoseStamped attitude_msg_;

    
public:

    CommandInterface(){};
    ~CommandInterface(){};

    void startPixhawkInterface();
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();

};

