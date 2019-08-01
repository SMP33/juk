#include <iostream>
#include <sstream>
#include <ros/ros.h>

#include "juk_msg/juk_dji_gps_msg.h"
#include "juk_msg/juk_dji_device_status_msg.h"
#include "juk_msg/juk_control_dji_msg.h"

template<typename T>
	class LoggerJuk
	{
	public:
		LoggerJuk(){		}
		T data;
		ros::Time upd_time;
		ros::Subscriber sub;
	};


template<typename T>
	class JukLogger
	{
	public:
		JukLogger(ros::NodeHandle* nh,std::string topic_name)
		{
			sub = nh->subscribe(topic_name, 1, &JukLogger::callback, this);
		}
		
		T data;
		ros::Time upd_time;
		ros::Subscriber sub;
		
		void callback(const typename T::ConstPtr& inp)
		{
			data = *inp;
			upd_time = ros::Time::now();
		}
	};