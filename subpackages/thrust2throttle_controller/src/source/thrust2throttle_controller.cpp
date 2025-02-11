#include "thrust2throttle_controller.hpp"


void Thrust2throttleController::setup()
{
 	static ros::NodeHandle nh;

	std::string n_space ;
	// std::string estimated_pose_topic;
	std::string imu_topic;
	std::string thrust_topic;
	std::string throttle_topic;
	std::string flight_state_topic;


	ros_utils_lib::getPrivateParam<double>	   ("~uav_mass"						, mass_						    ,1.0f);
	ros_utils_lib::getPrivateParam<std::string>("~namespace"					, n_space						,"drone1");
	// ros_utils_lib::getPrivateParam<std::string>("~estimated_pose_topic" 	    , estimated_pose_topic 			,"self_localization/pose");
	ros_utils_lib::getPrivateParam<std::string>("~imu_topic"	    			, imu_topic						,"sensor_measurement/imu");
	ros_utils_lib::getPrivateParam<std::string>("~thrust_topic"					, thrust_topic					,"actuator_command/thrust");
	ros_utils_lib::getPrivateParam<std::string>("~throttle_topic"				, throttle_topic				,"actuator_command/throttle");
	ros_utils_lib::getPrivateParam<std::string>("~flight_state_topic"		    , flight_state_topic    		,"self_localization/flight_state");

  	

  	// pose_sub_     = nh.subscribe("/" + n_space + "/" + estimated_pose_topic	,1, &Thrust2throttleController::poseCallback,this);
  	imu_sub_      = nh.subscribe("/" + n_space + "/" + imu_topic           	,1, &Thrust2throttleController::imuCallback,this);
	thrust_sub_   = nh.subscribe("/" + n_space + "/" + thrust_topic			,1, &Thrust2throttleController::thrustCallback,this);   
	flight_state_sub = nh.subscribe("/" + n_space + "/" + flight_state_topic,1,&Thrust2throttleController::flightStateCallback,this);

	throttle_pub_ = nh.advertise<mavros_msgs::Thrust>("/" + n_space + "/" + throttle_topic,1);

	#if DEBUG==1
		debug_signals_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/" + n_space + "/" + "debug/thrust2throttle/debug_signals",1);
		debug_signals_msg_.data = std::vector<float>(3,0.0f);
	#endif

}


void Thrust2throttleController::computeThrottle(){

	static float& throttle = throttle_msg_.thrust;
	
	float thrust_error = (accel_reference_- accel_measure_);
	#if DEBUG==1
		debug_signals_msg_.data[0] = accel_reference_;
		debug_signals_msg_.data[1] = accel_measure_;
		debug_signals_msg_.data[2] = thrust_error;
		debug_signals_pub_.publish(debug_signals_msg_);
	#endif
	// std::cout<< "accel_ref_ "<< accel_reference_ <<std::endl;  
	// std::cout<< "accel_measure_ "<< accel_measure_ <<std::endl;  
	// std::cout<< "thrust_error_ "<< thrust_error <<std::endl;  

	throttle = throttle + (Kp_ *thrust_error );
	 
	throttle = (throttle < 0)? 0 : throttle; // LOW LIMIT throttle IN [0, 1]
	throttle = (throttle > maximun_throttle)? maximun_throttle : throttle; // HIGH LIMIT throttle IN [0, 1]
	
	// std::cout<< "throttle "<< throttle <<std::endl;  

	publishThrottle();

}

void Thrust2throttleController::publishThrottle(){

	throttle_pub_.publish(throttle_msg_);
	
}

void Thrust2throttleController::imuCallback(const sensor_msgs::Imu& _msg){
	static float prev_measure = _msg.linear_acceleration.z;
	const float alpha = 0.2;
	// const float alpha = 1;
	accel_measure_ = prev_measure*(1-alpha) + _msg.linear_acceleration.z*alpha ;
	prev_measure = accel_measure_;

	// std::cout<< "accel_measure"<< accel_measure_ <<std::endl;
}

void Thrust2throttleController::thrustCallback(const mavros_msgs::Thrust& _msg){
	throttle_msg_.header = _msg.header;
	accel_reference_      = _msg.thrust/mass_;
	// std::cout<< "accel_reference"<< accel_reference_ <<std::endl;
	computeThrottle();

}
