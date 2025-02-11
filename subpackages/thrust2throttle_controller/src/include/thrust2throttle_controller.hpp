#pragma once
#include "ros/ros.h"
#include "mavros_msgs/Thrust.h"
#include "sensor_msgs/Imu.h"
#include <vector>
#include <iostream>
#include "tf/transform_datatypes.h"

#include "std_msgs/Float32MultiArray.h"
#include "aerostack_msgs/FlightState.h"

#include "ros_utils_lib/ros_utils.hpp"
#include "ros_utils_lib/control_utils.hpp"

#define DEBUG 1
#define GRAVITY_CONSTANT 9.81f
#define MAX_THROTTLE 1.0f
#define TAKEOFF_THROTTLE 1.0f


class Thrust2throttleController {
public:
    Thrust2throttleController(){};
    void setup();

private:
    
    double mass_ = 1.0f;
	const float Kp_ = 0.00125f, Kd_ = 0.0 , Ki = 0.000f;
	const float antiwindup_limit_ = 10;
    
    float maximun_throttle = 1.0f;
    

    ros::Subscriber imu_sub_; // dz reference¡    
    void imuCallback(const sensor_msgs::Imu & );
    float accel_measure_;
    
    ros::Subscriber thrust_sub_; // speed measures    
    void thrustCallback(const mavros_msgs::Thrust & );
    float accel_reference_;
    
    #if DEBUG==1
        ros::Publisher debug_signals_pub_; // pose
        std_msgs::Float32MultiArray debug_signals_msg_;
    // double roll_ = 0.0f, pitch_ = 0.0f;
    #endif

    ros::Subscriber flight_state_sub;
    aerostack_msgs::FlightState flight_state_msg_;

    void flightStateCallback(const aerostack_msgs::FlightState& _msg){
        static bool already_flying = false;
        flight_state_msg_ = _msg;
        // std::cout << "state = " << flight_state_msg_.state<< std::endl;
        
        if (!already_flying){
            if (flight_state_msg_.state == aerostack_msgs::FlightState::TAKING_OFF ||
                flight_state_msg_.state == aerostack_msgs::FlightState::FLYING 
            ){
                already_flying = true;
                std::cout << "FLIGHT BEGGINS"<<std::endl;
            }
            else{
                maximun_throttle = 0.01f;
            }

        }else{

            if (flight_state_msg_.state == aerostack_msgs::FlightState::FLYING   ||
                flight_state_msg_.state == aerostack_msgs::FlightState::HOVERING){
                maximun_throttle = MAX_THROTTLE;
                
            }
            else
                maximun_throttle = TAKEOFF_THROTTLE;
        }
            

	};

    
    ros::Publisher throttle_pub_; // thrust_pub
    mavros_msgs::Thrust throttle_msg_;

    
    void computeThrottle();
    void publishThrottle();

    
};
