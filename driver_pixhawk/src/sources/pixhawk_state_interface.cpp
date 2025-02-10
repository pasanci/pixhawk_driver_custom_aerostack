#include "pixhawk_state_interface.hpp"

void StateInterface::ownSetUp(){
    
    
    // read params from launcher
    ros_utils_lib::getPrivateParam<std::string>("~namespace"					    , n_space_						    ,"drone1");
    //subscribers
    ros_utils_lib::getPrivateParam<std::string>("~mavros_battery_topic"		        , mavros_battery_topic_             ,"mavros/battery");
    ros_utils_lib::getPrivateParam<std::string>("~mavros_imu_topic"				    , mavros_imu_topic_                 ,"mavros/imu/data");
	ros_utils_lib::getPrivateParam<std::string>("~mavros_altitude_topic"		    , mavros_altitude_topic_            ,"mavros/altitude");
    ros_utils_lib::getPrivateParam<std::string>("~mavros_magnetometer_topic"	    , mavros_magnetometer_topic_        ,"mavros/imu/mag");
	ros_utils_lib::getPrivateParam<std::string>("~mavros_odom_topic"			    , mavros_odom_topic_                ,"mavros/local_position/odom");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_estimated_pose_topic"   , aerostack_estimated_pose_topic_   ,"self_localization/pose");
	ros_utils_lib::getPrivateParam<std::string>("~aerostack_estimated_speed_topic"  , aerostack_estimated_speed_topic_  ,"self_localization/speed");

    //publishers
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_battery_topic"          , aerostack_battery_topic_ 		    ,"sensor_measurement/battery_state");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_imu_topic"              , aerostack_imu_topic_ 		        ,"sensor_measurement/imu");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_altitude_topic"         , aerostack_altitude_topic_ 	    ,"sensor_measurement/altitude");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_magnetometer_topic"     , aerostack_magnetometer_topic_     ,"sensor_measurement/magnetometer");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_odom_topic"             , aerostack_odom_topic_ 		    ,"sensor_measurement/odometry");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_linear_speed_topic"     , aerostack_linear_speed_topic_     ,"sensor_measurement/linear_speed");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_rotation_angles_topic"  , aerostack_rotation_angles_topic_ 	,"sensor_measurement/rotation_angles");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_flight_state_topic"     , aerostack_flight_state_topic_ 	,"self_localization/flight_state");
    ros_utils_lib::getPrivateParam<std::string>("~aerostack_flight_action_command_topic"     , aerostack_flight_action_command_topic_ 	,"actuator_command/flight_action");

    ros_utils_lib::getPrivateParam<std::string>("~mavros_vision_pose_topic_"        , mavros_vision_pose_topic_ 	    ,"mavros/vision_pose/pose");


}

void StateInterface::poseCallback(const geometry_msgs::PoseStamped& msg){
   static ros::Time time = ros::Time::now();
   static geometry_msgs::PoseStamped msg_NED; 
   float freq = 30;
   if ((ros::Time::now()-time).toSec()>1/freq){
        
        // Convert from FLU (aerostack) to ENU (mavros)
        msg_NED.pose.position.x = - msg.pose.position.y;
        msg_NED.pose.position.y = msg.pose.position.x;

        msg_NED.header.stamp = ros::Time::now();

        mavros_vision_pose_pub_.publish(msg_NED);
        time = ros::Time::now();
   }
}

void StateInterface::CallbackAltitudeTopic(const mavros_msgs::Altitude& msg){
    static geometry_msgs::PointStamped altitude_msg;
    altitude_msg.header  = msg.header;
    altitude_msg.point.z = -msg.relative;
    altitude_pub_.publish(altitude_msg);
}


void StateInterface::CallbackMagnetometerTopic(const sensor_msgs::MagneticField& msg){
    static geometry_msgs::Vector3Stamped magnetometer_msg;
    magnetometer_msg.header  = msg.header;
    magnetometer_msg.vector = msg.magnetic_field;
    magnetometer_pub_.publish(magnetometer_msg);
}

void StateInterface::CallbackOdomTopic(const nav_msgs::Odometry& msg){
    static geometry_msgs::TwistStamped vel_msg;
    vel_msg.header = msg.header;
    vel_msg.twist = msg.twist.twist;
    odom_pub_.publish(msg);
    linear_speeds_pub_.publish(vel_msg);
}


void StateInterface::ownStart(){

    battery_sub_               = nh_.subscribe("/" + n_space_ + "/" + mavros_battery_topic_             , 10, &StateInterface::CallbackBatteryTopic,this);
    imu_sub_                   = nh_.subscribe("/" + n_space_ + "/" + mavros_imu_topic_                 , 10, &StateInterface::CallbackImuTopic,this);
    altitude_sub_              = nh_.subscribe("/" + n_space_ + "/" + mavros_altitude_topic_            , 10, &StateInterface::CallbackAltitudeTopic,this);
    magnetometer_sub_          = nh_.subscribe("/" + n_space_ + "/" + mavros_magnetometer_topic_        , 10, &StateInterface::CallbackMagnetometerTopic,this);
    odom_sub_                  = nh_.subscribe("/" + n_space_ + "/" + mavros_odom_topic_                , 10, &StateInterface::CallbackOdomTopic,this);
    estimated_pose_sub_        = nh_.subscribe("/" + n_space_ + "/" + aerostack_estimated_pose_topic_   , 10, &StateInterface::poseCallback,this);
    estimated_twist_sub_       = nh_.subscribe("/" + n_space_ + "/" + aerostack_estimated_speed_topic_  , 10, &StateInterface::twistCallback,this);
    flight_action_command_sub_ = nh_.subscribe("/" + n_space_ + "/" + aerostack_flight_action_command_topic_ , 10, &StateInterface::flightActionCommandCallback,this);
    flightstate_sub            = nh_.subscribe("/" + n_space_ + "/" + aerostack_flight_state_topic_     , 1 , &StateInterface::statusCallBack, this);
    
    

    battery_pub_            = nh_.advertise<sensor_msgs::BatteryState>       ("/" + n_space_ + "/" + aerostack_battery_topic_        , 1, true);
    imu_pub_                = nh_.advertise<sensor_msgs::Imu>                ("/" + n_space_ + "/" + aerostack_imu_topic_            , 1, true);
    altitude_pub_           = nh_.advertise<geometry_msgs::PointStamped>     ("/" + n_space_ + "/" + aerostack_altitude_topic_       , 1, true);
    rotation_angles_pub_    = nh_.advertise<geometry_msgs::Vector3Stamped>   ("/" + n_space_ + "/" + aerostack_rotation_angles_topic_, 1, true);
    magnetometer_pub_       = nh_.advertise<geometry_msgs::Vector3Stamped>   ("/" + n_space_ + "/" + aerostack_magnetometer_topic_   , 1, true);
    odom_pub_               = nh_.advertise<nav_msgs::Odometry>              ("/" + n_space_ + "/" + aerostack_odom_topic_           , 1, true);
    linear_speeds_pub_      = nh_.advertise<geometry_msgs::TwistStamped>     ("/" + n_space_ + "/" + aerostack_linear_speed_topic_   , 1, true);
    flight_state_pub_       = nh_.advertise<aerostack_msgs::FlightState>     ("/" + n_space_ + "/" + aerostack_flight_state_topic_   , 1, true);

    mavros_vision_pose_pub_       = nh_.advertise<geometry_msgs::PoseStamped>     ("/" + n_space_ + "/" + mavros_vision_pose_topic_   , 1, true);

}


void StateInterface::getFlightState(){


    switch(flight_action_command_msg_.action){
        
        case aerostack_msgs::FlightActionCommand::TAKE_OFF:
            if (flight_state_msg_.state == aerostack_msgs::FlightState::LANDED || flight_state_msg_.state == aerostack_msgs::FlightState::UNKNOWN){
                flight_state_msg_.state = aerostack_msgs::FlightState::TAKING_OFF;
            }
            else{
                if (flight_state_msg_.state == aerostack_msgs::FlightState::TAKING_OFF){
                    if (pose_msg_.pose.position.z > 0.1){
                        flight_state_msg_.state = aerostack_msgs::FlightState::FLYING;
                    }
                }
            }
        break;
        case aerostack_msgs::FlightActionCommand::HOVER:{
            if(pose_msg_.pose.position.z > 0.1 && std::abs(twist_msg_.twist.linear.x) < 0.05 && std::abs(twist_msg_.twist.linear.y) < 0.05 && std::abs(twist_msg_.twist.linear.z) < 0.05 &&
            std::abs(twist_msg_.twist.angular.x) < 0.05 && std::abs(twist_msg_.twist.angular.y) < 0.05 && std::abs(twist_msg_.twist.angular.z) < 0.05){
                flight_state_msg_.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
        break;
        case aerostack_msgs::FlightActionCommand::LAND:{
            if (flight_state_msg_.state != aerostack_msgs::FlightState::LANDED && flight_state_msg_.state != aerostack_msgs::FlightState::LANDING){
                flight_state_msg_.state = aerostack_msgs::FlightState::LANDING;
                landing_command_time_ = ros::Time::now();
            }
            else{
                if (flight_state_msg_.state == aerostack_msgs::FlightState::LANDING){
                    if (fabs(twist_msg_.twist.linear.z) > 0.05){
                        landing_command_time_ = ros::Time::now();
                    }
                    else if (((ros::Time::now()-landing_command_time_).toSec() > LANDING_CHECK_DELAY)){
                        flight_state_msg_.state = aerostack_msgs::FlightState::LANDED;
                    }
                }
            }
        }
        break;
        case aerostack_msgs::FlightActionCommand::MOVE:{
            if(std::abs(pose_msg_.pose.position.z) > 0.1 && (std::abs(twist_msg_.twist.linear.x) > 0.05 || std::abs(twist_msg_.twist.linear.y) > 0.05 || std::abs(twist_msg_.twist.linear.z) > 0.05 ||
            std::abs(twist_msg_.twist.angular.x) > 0.05 || std::abs(twist_msg_.twist.angular.y) > 0.05 || std::abs(twist_msg_.twist.angular.z) > 0.05)){
                flight_state_msg_.state = aerostack_msgs::FlightState::FLYING;
            }
            if(std::abs(pose_msg_.pose.position.z) > 0.1 && std::abs(twist_msg_.twist.linear.x) < 0.05 && std::abs(twist_msg_.twist.linear.y) < 0.05 && std::abs(twist_msg_.twist.linear.z) < 0.05 &&
            std::abs(twist_msg_.twist.angular.x) < 0.05 && std::abs(twist_msg_.twist.angular.y) < 0.05 && std::abs(twist_msg_.twist.angular.z) < 0.05){
                flight_state_msg_.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
        break;
        case aerostack_msgs::FlightActionCommand::UNKNOWN:
        default:{
            if(pose_msg_.pose.position.z < 0.1 && std::abs(twist_msg_.twist.linear.x) < 0.05 && std::abs(twist_msg_.twist.linear.y) < 0.05 && std::abs(twist_msg_.twist.linear.z) < 0.05 &&
            std::abs(twist_msg_.twist.angular.x) < 0.05 && std::abs(twist_msg_.twist.angular.y) < 0.05 && std::abs(twist_msg_.twist.angular.z) < 0.05){
                flight_state_msg_.state = aerostack_msgs::FlightState::LANDED;
            }
            if(pose_msg_.pose.position.z > 0.1 && (std::abs(twist_msg_.twist.linear.x) > 0.05 || std::abs(twist_msg_.twist.linear.y) > 0.05 || std::abs(twist_msg_.twist.linear.z) > 0.05 ||
            std::abs(twist_msg_.twist.angular.x) > 0.05 || std::abs(twist_msg_.twist.angular.y) > 0.05 || std::abs(twist_msg_.twist.angular.z) > 0.05)){
                flight_state_msg_.state = aerostack_msgs::FlightState::FLYING;
            }
            if(pose_msg_.pose.position.z > 0.1 && std::abs(twist_msg_.twist.linear.x) < 0.05 && std::abs(twist_msg_.twist.linear.y) < 0.05 && std::abs(twist_msg_.twist.linear.z) < 0.05 &&
            std::abs(twist_msg_.twist.angular.x) < 0.05 && std::abs(twist_msg_.twist.angular.y) < 0.05 && std::abs(twist_msg_.twist.angular.z) < 0.05){
                flight_state_msg_.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
        break;
    }

}

void StateInterface::ownStop(){

    battery_sub_.shutdown();
    imu_sub_.shutdown();
    altitude_sub_.shutdown();
    magnetometer_sub_.shutdown();
    odom_sub_.shutdown();
    estimated_pose_sub_.shutdown();
    estimated_twist_sub_.shutdown();
    battery_pub_.shutdown();
    imu_pub_.shutdown();
    altitude_pub_.shutdown();
    rotation_angles_pub_.shutdown();
    magnetometer_pub_.shutdown();
    odom_pub_.shutdown();
    linear_speeds_pub_.shutdown();
    flight_state_pub_.shutdown();

}

void StateInterface::ownRun(){
    getFlightState();
    flight_state_msg_.header.stamp = ros::Time::now();
    flight_state_pub_.publish(flight_state_msg_);

}

void StateInterface::CallbackImuTopic(const sensor_msgs::Imu& _msg){
    imu_msg_ = _msg;
    tf::Quaternion q;
    tf::quaternionMsgToTF(_msg.orientation,q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);

    imu_msg_.orientation=  tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);

    imu_pub_.publish(imu_msg_);

    geometry_msgs::Vector3Stamped rotation_angles_msg;
    rotation_angles_msg.header = _msg.header;

    //convert quaternion msg to eigen

    // TODO: CLEAN ALL OF THIS

//------ The below code converts ENU (Mavros) frame to NED (Aerostack) frame ------- ///
    
    // //Rotating the frame in x-axis by 180 degrees
    // Eigen::Quaterniond BASE_LINK_TO_AIRCRAFT = tf::createQuaternionFromRPY(M_PI, 0.0, 0.0);
    // quaterniond = quaterniond*BASE_LINK_TO_AIRCRAFT;


    //Rotating the frame in x-axis by 180 deg and in z-axis by 90 axis (accordance to the new mavros update)
    Eigen::Quaterniond quaternion;
    tf::quaternionMsgToEigen(_msg.orientation, quaternion);
    auto quat_tf = tf::createQuaternionFromRPY(M_PI, 0.0, M_PI_2);
    Eigen::Quaterniond enu_to_ned_quat(quat_tf.getX(),quat_tf.getY(),quat_tf.getZ(),quat_tf.getW());
    quaternion = enu_to_ned_quat*quaternion;

//-----------------------------------------------------------------------------------///

    auto euler_angles_vector = quaternion.toRotationMatrix().eulerAngles(0,1,2);
    // euler_angles_vector = [roll, pitch, yaw] 

    rotation_angles_msg.vector.x =   euler_angles_vector[0];
    rotation_angles_msg.vector.y =   euler_angles_vector[1];
    rotation_angles_msg.vector.z = - euler_angles_vector[2]; //CHECK - sign
    

    rotation_angles_pub_.publish(rotation_angles_msg);
};

// void StateInterface::CallbackRotationAngles(const )


void StateInterface::CallbackBatteryTopic(const sensor_msgs::BatteryState& _msg){
    battery_msg_ = _msg;
    battery_pub_.publish(battery_msg_);
};

void StateInterface::statusCallBack(const aerostack_msgs::FlightState &msg){
  flight_state_msg_.state = msg.state;
}