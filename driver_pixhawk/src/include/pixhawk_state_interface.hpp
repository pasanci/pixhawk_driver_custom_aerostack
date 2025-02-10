#pragma once

#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/PoseStamped.h"
#include <robot_process.h>

#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

#include "Eigen/Dense"
#include <tf/transform_datatypes.h>
#include "mavros/mavros_uas.h"
#include "aerostack_msgs/FlightState.h"
#include "aerostack_msgs/FlightActionCommand.h"

#include "ros_utils_lib/ros_utils.hpp"
#include <iostream>

#define LANDING_CHECK_DELAY 1

class StateInterface : public RobotProcess{

private:
    ros::NodeHandle nh_;


    std::string n_space_;
    // subscribers
    std::string mavros_battery_topic_;
    std::string mavros_imu_topic_;
    std::string mavros_altitude_topic_;
    std::string mavros_magnetometer_topic_;
    std::string mavros_odom_topic_;
    std::string aerostack_estimated_pose_topic_;
    std::string aerostack_estimated_speed_topic_;
    

    
    // publishers
    std::string aerostack_battery_topic_;
    std::string aerostack_imu_topic_;
    std::string aerostack_altitude_topic_;
    std::string aerostack_magnetometer_topic_;
    std::string aerostack_odom_topic_;
    std::string aerostack_linear_speed_topic_;
    std::string aerostack_rotation_angles_topic_;
    std::string aerostack_flight_state_topic_;
    std::string aerostack_flight_action_command_topic_;

    std::string mavros_vision_pose_topic_;

    ros::Time landing_command_time_;

    // Battery
    ros::Publisher  battery_pub_;
    ros::Subscriber battery_sub_; 
    sensor_msgs::BatteryState battery_msg_;    
    void CallbackBatteryTopic(const sensor_msgs::BatteryState& );

    // Imu 
    ros::Publisher  imu_pub_;
    ros::Publisher  rotation_angles_pub_;
    ros::Subscriber imu_sub_; 
    sensor_msgs::Imu imu_msg_;    
    void CallbackImuTopic(const sensor_msgs::Imu& );
    
    ros::Publisher  altitude_pub_;
    ros::Subscriber altitude_sub_;
    void CallbackAltitudeTopic(const mavros_msgs::Altitude& msg);
    
    ros::Publisher  magnetometer_pub_;
    ros::Subscriber magnetometer_sub_;
    void CallbackMagnetometerTopic(const sensor_msgs::MagneticField& msg);

    ros::Subscriber odom_sub_;
    ros::Publisher  odom_pub_;
    ros::Publisher  linear_speeds_pub_;
    void CallbackOdomTopic(const nav_msgs::Odometry& msg);

    ros::Subscriber  flight_action_command_sub_;
    aerostack_msgs::FlightActionCommand flight_action_command_msg_; 

    void flightActionCommandCallback(const aerostack_msgs::FlightActionCommand& msg){
        flight_action_command_msg_ = msg;
    };
    void getFlightState();

    aerostack_msgs::FlightState flight_state_msg_; 

    ros::Subscriber estimated_pose_sub_;
    ros::Publisher mavros_vision_pose_pub_;
    void poseCallback(const geometry_msgs::PoseStamped& );
  
    geometry_msgs::PoseStamped pose_msg_;

    ros::Subscriber estimated_twist_sub_;
    void twistCallback(const geometry_msgs::TwistStamped& _msg){twist_msg_ = _msg;};
    geometry_msgs::TwistStamped twist_msg_;

    ros::Publisher flight_state_pub_;
    ros::Subscriber flightstate_sub;


public:

    StateInterface(){};
    ~StateInterface(){};

    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();

    void statusCallBack(const aerostack_msgs::FlightState &msg);
};

