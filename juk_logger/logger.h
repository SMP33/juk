#include <iostream>
#include <sstream>
#include <ros/ros.h>




template<typename T>
	class SimpleSub
	{
	public:
		SimpleSub(ros::NodeHandle* nh, std::string topic_name_, ros::Time start_time_)
			:
			topic_name(topic_name_),
			start_time(start_time_),
			upd(false)
		{
			sub = nh->subscribe(topic_name, 1, &SimpleSub::callback, this);
			last_upd_time = ros::Time::now();
		}
		
		std::string topic_name;
		T data;
		
		ros::Time last_upd_time;
		ros::Time start_time;
		bool upd;
		void clean_upd()
		{
			upd = false;
		}
		
		ros::Subscriber sub;
		
		std::stringstream full_str;
		
		std::string get_full_str()
		{
			return full_str.str()+"\n";
		}
		void reset_str()
		{
			full_str = std::stringstream();
			full_str.flags(std::ios::fixed);
			full_str.precision(10);
			full_str << "@TOPIC:[" << topic_name <<"]"<< std::endl;
			full_str << "\tUPD:[" << (upd ? "YES" : "NO") << "]" <<std::endl; 
			full_str << "\tLAST_UPD:[" << (last_upd_time - start_time) << "]" << std::endl;
			full_str << std::endl;
		}
		template<typename V>
			void add_str(std::string txt, V value)
			{
				full_str << "\t" << txt << ":[" << value << "]" << std::endl;
			}
		
		void callback(const typename T::ConstPtr& inp)
		{
			data = *inp;
			last_upd_time = ros::Time::now();
			upd = true;
		}
	};