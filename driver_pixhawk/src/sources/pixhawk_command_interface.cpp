#include "pixhawk_command_interface.hpp"


void CommandInterface::ownSetUp(){
       
    // read params from launcher
    ros_utils_lib::getPrivateParam<std::string>("~namespace" , n_space_ ,"drone1");
    ros_utils_lib::getPrivateParam<bool>("~simulation_mode" , simulation_mode_, true);
    ros_utils_lib::getPrivateParam<bool>("~acro_mode" , acro_mode_, false);

    //subscribers
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_flight_action_command_topic"                , aerostack_flight_action_command_topic_               , "actuator_command/flight_action");
    ros_utils_lib::getPrivateParam<std::string>("~mavros_state_topic"                 				    , mavros_state_topic_                                  , "mavros/state");
    ros_utils_lib::getPrivateParam<std::string>("~mavros_imu_topic"                 				    , mavros_imu_topic_                                    , "mavros/imu/data");

	ros_utils_lib::getPrivateParam<std::string>("~aerostack_roll_pitch_yaw_rate_thrust_command_topic"   , aerostack_roll_pitch_yaw_rate_thrust_command_topic_  , "actuator_command/roll_pitch_yaw_rate_thrust");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_speed_command_topic"                        , aerostack_speed_command_topic_                       , "actuator_command/speed");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_pose_command_topic"                         , aerostack_pose_command_topic_                        , "actuator_command/pose");
	ros_utils_lib::getPrivateParam<std::string>("~aerostack_thottle_command_topic"                    	, aerostack_thottle_command_topic_                     , "actuator_command/throttle");

    ros_utils_lib::getPrivateParam<std::string>("~mavros_attitude_command_topic"    , mavros_attitude_command_topic_     ,"mavros/setpoint_attitude/attitude");
	ros_utils_lib::getPrivateParam<std::string>("~mavros_cmd_vel_command_topic"     , mavros_cmd_vel_command_topic_      ,"mavros/setpoint_attitude/cmd_vel");
	ros_utils_lib::getPrivateParam<std::string>("~mavros_thrust_command_topic"      , mavros_thrust_command_topic_       ,"mavros/setpoint_attitude/thrust");

	ros_utils_lib::getPrivateParam<std::string>("~mavros_arming_service"            , mavros_arming_service_            ,"mavros/cmd/arming");
	ros_utils_lib::getPrivateParam<std::string>("~mavros_takeoff_service"           , mavros_takeoff_service_           ,"mavros/cmd/takeoff");
	ros_utils_lib::getPrivateParam<std::string>("~mavros_landing_service"           , mavros_landing_service_           ,"mavros/cmd/landing");
	ros_utils_lib::getPrivateParam<std::string>("~mavros_set_mode_service"          , mavros_set_mode_service_          ,"mavros/set_mode");

}



void CommandInterface::ownStart(){

    arming_client_          = nh_.serviceClient<mavros_msgs::CommandBool>   ("/" + n_space_ + "/" + mavros_arming_service_);
    takeoff_client_         = nh_.serviceClient<mavros_msgs::CommandTOL>    ("/" + n_space_ + "/" + mavros_takeoff_service_);
    landing_client_         = nh_.serviceClient<mavros_msgs::CommandTOL>    ("/" + n_space_ + "/" + mavros_landing_service_);
    set_mode_client_        = nh_.serviceClient<mavros_msgs::SetMode>       ("/" + n_space_ + "/" + mavros_set_mode_service_);

    flight_commands_sub_    = nh_.subscribe("/" + n_space_ + "/" + aerostack_flight_action_command_topic_               ,1, &CommandInterface::flightActionCallback,     this);
    state_sub_              = nh_.subscribe("/" + n_space_ + "/" + mavros_state_topic_                                  ,1, &CommandInterface::UAVStateCallback,         this);
    deprecated_attitude_thurst_sub_    = nh_.subscribe("/" + n_space_ + "/" + aerostack_roll_pitch_yaw_rate_thrust_command_topic_  ,1, &CommandInterface::alttitudeThrustCallback,  this);
    
    atittude_sub_           = nh_.subscribe("/" + n_space_ + "/" + aerostack_pose_command_topic_                        ,1, &CommandInterface::attitudeCallback,       this);
    datittude_sub_          = nh_.subscribe("/" + n_space_ + "/" + aerostack_speed_command_topic_                       ,1, &CommandInterface::dAttitudeCallback,       this);
    throttle_sub_           = nh_.subscribe("/" + n_space_ + "/" + aerostack_thottle_command_topic_                     ,1, &CommandInterface::thrustCallback,           this);
    imu_sub_                = nh_.subscribe("/" + n_space_ + "/" + mavros_imu_topic_                                    ,1, &CommandInterface::CallbackImuTopic,         this);

    attitude_setpoint_pub_  = nh_.advertise<geometry_msgs::PoseStamped>  ("/" + n_space_ + "/" + mavros_attitude_command_topic_ , 1);
    dattitude_setpoint_pub_ = nh_.advertise<geometry_msgs::TwistStamped> ("/" + n_space_ + "/" + mavros_cmd_vel_command_topic_  , 1);
    thrust_setpoint_pub_    = nh_.advertise<mavros_msgs::Thrust>         ("/" + n_space_ + "/" + mavros_thrust_command_topic_   , 1);
    
    thrust_msg_.thrust = 0.00f;

    dattitude_msg_.twist.angular.x = 0.0f;
    dattitude_msg_.twist.angular.y = 0.0f;
    dattitude_msg_.twist.angular.z = 0.0f;

    attitude_msg_.pose.orientation.w = 1.0f;

    startPixhawkInterface();
}

#define YAW_STABILIZATION_STEPS 100

void CommandInterface::CallbackImuTopic(const sensor_msgs::Imu& _msg){
    static unsigned short n_iter = 0; 
    if (initial_yaw_fixed_) return;
    else if (n_iter < YAW_STABILIZATION_STEPS){
        n_iter++;
        tf::Quaternion q(_msg.orientation.x,_msg.orientation.y,_msg.orientation.z,_msg.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        initial_yaw = yaw;
    }else{
        initial_yaw_fixed_ = true;
    }

};



void CommandInterface::checkOffboardMode(){
    static mavros_msgs::SetMode offb_set_mode;
    static bool first_time = true;
    
    offb_set_mode.request.custom_mode = "OFFBOARD";
    static auto last_request = ros::Time::now();
    if (first_time){
        first_time = false;
            if( set_mode_client_.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    }

    if(current_state_.mode != "OFFBOARD"){
        if((ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client_.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
    }
}


void CommandInterface::startPixhawkInterface(){

    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state_.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::Thrust thrust_setpoint;
    thrust_setpoint.thrust = 0.00f;
    
    geometry_msgs::TwistStamped dattitude;
    dattitude.twist.angular.x = 0.0f;
    dattitude.twist.angular.y = 0.0f;
    dattitude.twist.angular.z = 0.0f;
	
    //send a few setpoints before starting
    std::cout<< "Sending initial setpoints"<< std::endl;

    for(int i = 100; ros::ok() && i > 0; --i){
        thrust_setpoint_pub_.publish(thrust_setpoint);
        if (acro_mode_){
            dattitude_setpoint_pub_.publish(dattitude);
        }
        else{
            attitude_setpoint_pub_.publish(attitude_msg_);
        }

        ros::spinOnce();
        rate.sleep();
    }

    if (!simulation_mode_)
        std::cout<< "PIXHAWK connected, put RC in OFFBOARDMODE" << std::endl;
    else
        std::cout<< "PIXHAWK connected, in simulation mode" << std::endl;

    if (acro_mode_)
        std::cout<< "PIXHAWK configured in ACRO mode" << std::endl;
    else
        std::cout<< "PIXHAWK configured in ATTITUDE mode" << std::endl;
        
    
}

 

void CommandInterface::ownStop(){

    flight_commands_sub_.shutdown();
    state_sub_.shutdown();
    deprecated_attitude_thurst_sub_.shutdown();
    datittude_sub_.shutdown();
    throttle_sub_.shutdown();
    attitude_setpoint_pub_.shutdown();
    dattitude_setpoint_pub_.shutdown();
    thrust_setpoint_pub_.shutdown();
}

void CommandInterface::ownRun(){
    static auto timestamp = ros::Time::now();
    timestamp = ros::Time::now();
    
    if (acro_mode_){
        dattitude_msg_.header.stamp = timestamp;
        dattitude_msg_.header.frame_id = "base_link";
        dattitude_setpoint_pub_.publish(dattitude_msg_);
    }
    else{
        attitude_msg_.header.stamp = timestamp;
        attitude_msg_.header.frame_id = "odom";
        attitude_setpoint_pub_.publish(attitude_msg_);
    }

    throttle_msg_.header.stamp = timestamp;
    thrust_setpoint_pub_.publish(throttle_msg_);
    
    if (simulation_mode_){
        this->checkOffboardMode();
    }

}

void CommandInterface::UAVStateCallback(const mavros_msgs::State& _msg){
    current_state_ = _msg;
}

bool CommandInterface::armVehicle(const bool& _value){

    if (current_state_.armed == _value){
        if(_value)
            ROS_INFO("Vehicle is already armed");
        else
            ROS_INFO("Vehicle is already disarmed");
        return true; 
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = _value;
    if(current_state_.mode == "OFFBOARD"){
        if(arming_client_.call(arm_cmd) && arm_cmd.response.success){
            if(_value)
                ROS_INFO("Vehicle armed");
            else
                ROS_INFO("Vehicle disarmed");
            return true;
        }
        else{
            if(_value)
                ROS_WARN("ERROR arming vehicle");
            else
                ROS_WARN("ERROR disarming vehicle");
        }
    }
    
    return false;
}

void CommandInterface::thrustCallback(const mavros_msgs::Thrust& msg){
    throttle_msg_ = msg;
}

float checkYawBounds(float yaw){
    if (yaw > 2*M_PI)
        yaw = 2 * M_PI - yaw;
    else if (yaw < 0.0f)
        yaw = yaw + 2 * M_PI;
    return yaw;
}

void CommandInterface::alttitudeThrustCallback(const mav_msgs::RollPitchYawrateThrust& msg){
    #if USE_DEPRECATED_CONTROL_TOPICS

    if (!initial_yaw_fixed_) return;

    thrust_msg_.header = msg.header;
    thrust_msg_.thrust = msg.thrust.z;
    
    attitude_msg_.header = msg.header;

    static float yaw = 0.0f;
    // static float yaw = initial_yaw;
    
    static ros::Time prev_time = ros::Time::now();
    yaw = yaw + msg.yaw_rate* (ros::Time::now()-prev_time).toSec();
    prev_time = ros::Time::now();

    yaw = checkYawBounds(yaw);
    
    // auto q = tf::createQuaternionFromRPY(msg.roll,msg.pitch,msg.yaw_rate); //TODO
    // Take in considering initial yaw to avoid abrupt movement in yaw in takeoff
    auto q = tf::createQuaternionFromRPY(msg.roll,msg.pitch,yaw); //TODO
    
    attitude_msg_.pose.orientation.x = q.getX();
    attitude_msg_.pose.orientation.y = q.getY();
    attitude_msg_.pose.orientation.z = q.getZ();
    attitude_msg_.pose.orientation.w = q.getW();
    
    #endif
    
}
void CommandInterface::attitudeCallback(const geometry_msgs::PoseStamped &msg){
    attitude_msg_ = msg;
    attitude_msg_.header.stamp = ros::Time::now();
    attitude_msg_.header.frame_id = "base_link";
    
}

void CommandInterface::dAttitudeCallback(const geometry_msgs::TwistStamped &msg){
    dyaw = msg.twist.angular.z;
    dattitude_msg_ = msg;
}


void CommandInterface::flightActionCallback(const aerostack_msgs::FlightActionCommand& _msg)
{
    static aerostack_msgs::FlightActionCommand flight_action_msg = _msg;
    switch(_msg.action){
    case aerostack_msgs::FlightActionCommand::TAKE_OFF:{
        this->armVehicle(true);
        }
        break;
    case aerostack_msgs::FlightActionCommand::LAND:{
        // this->armVehicle(false);
        }
        break;
    case aerostack_msgs::FlightActionCommand::HOVER:
        // TODO::HOVERING 
        break;
    default:
        break;
    }
}
