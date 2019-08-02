#include <iostream>
#include <sstream>
#include <ros/ros.h>




template<typename T>
	class SimpleSub
	{
	public:
		SimpleSub(ros::NodeHandle* nh,std::string topic_name):
			name(topic_name)
		{
			sub = nh->subscribe(topic_name, 1, &SimpleSub::callback, this);
			upd_time = ros::Time::now();
		}
		
		std::string name;
		T data;
		ros::Time upd_time;
		ros::Subscriber sub;
		
		std::stringstream full_str;
		
		std::string get_full_str()
		{
			return full_str.str() +  "$TOPIC\n\n";
		}
		void reset_str()
		{
			full_str = std::stringstream();
			full_str.flags(std::ios::fixed);
			full_str.precision(10);
			full_str << "@TOPIC:[" << name <<"]"<< std::endl;
			full_str << "\tLAST_UPD:[" << upd_time << "]" << std::endl;
		}
		template<typename V>
			void add_str(std::string txt, V value)
			{
				full_str << "\t" << txt << ":[" << value << "]" << std::endl;
			}
		
		void callback(const typename T::ConstPtr& inp)
		{
			data = *inp;
			upd_time = ros::Time::now();
		}
	};