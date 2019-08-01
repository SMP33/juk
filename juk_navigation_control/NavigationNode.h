#include <ros/ros.h>

#include <GeoMath.h>

#include "juk_msg/juk_dji_gps_msg.h"
#include "juk_msg/juk_dji_device_status_msg.h"
#include "juk_msg/juk_control_dji_msg.h"
#include "juk_msg/juk_set_target_data_msg.h"
#include "juk_msg/juk_position_data_msg.h"

#include "std_msgs/String.h"


class NavigationNode
{
public:
	NavigationNode();
	

	

private:
	void gps_callback(const juk_msg::juk_dji_gps_msg::ConstPtr& input);
	void set_target_callback(const juk_msg::juk_set_target_data_msg::ConstPtr& input);
	ros::Time node_start_time;
	bool set_homepoint_flag = true;
	
	GeoMath::v3geo	homepoint;
	GeoMath::v3geo	current_point_abs;
	GeoMath::v3		current_velocity;
	GeoMath::v3     velocity_need;
	GeoMath::v3     current_point_home;
	
	struct 
	{
		GeoMath::v3geo	point_abs;
		uint8_t break_mode;
		float cruising_speed;
		float accurancy;
	}target;
	
	ros::NodeHandle nh;
	ros::Publisher pub_dji_control;
	ros::Publisher pub_position_data;
	ros::Subscriber sub_dji_gps;
	ros::Subscriber sub_set_target;
	ros::Publisher pub_str;
	
	uint8_t ctrl_mode;
	
	uint8_t ctrl_flag;
	
	void calculateVelocity(double abs_speed, GeoMath::v3 offset, GeoMath::v3 current_velocity, uint8_t ctrl_mode);

};

NavigationNode::NavigationNode()
{
	node_start_time = ros::Time::now();
	pub_dji_control = nh.advertise<juk_msg::juk_control_dji_msg>("JUK/CONTROL_DJI", 1);
	pub_position_data = nh.advertise<juk_msg::juk_position_data_msg>("JUK/POSITION_DATA", 1);
	pub_str = nh.advertise<std_msgs::String>("JUK/STR", 1);
	
	target.cruising_speed = 3;
	ctrl_mode = juk_msg::juk_set_target_data_msg::mode_allow_break_distance;

	sub_dji_gps = nh.subscribe("JUK/DJI/GPS", 1, &NavigationNode::gps_callback, this);
	sub_set_target = nh.subscribe("JUK/TARGET", 1, &NavigationNode::set_target_callback, this);
}

void
NavigationNode::gps_callback(const juk_msg::juk_dji_gps_msg::ConstPtr& input)
{


	
	auto now = ros::Time::now();
	if (set_homepoint_flag&&(now - node_start_time).toNSec() > 500000000)
	{
		homepoint = GeoMath::v3geo(input->lat*GeoMath::CONST.RAD2DEG, input->lng*GeoMath::CONST.RAD2DEG, input->alt);
		target.point_abs = homepoint;
		
		set_homepoint_flag = false;
		
		std::cout << "HOMEPOINT SET" << std::endl;
	}
	
	current_point_abs =  GeoMath::v3geo(input->lat*GeoMath::CONST.RAD2DEG, input->lng*GeoMath::CONST.RAD2DEG, input->alt);
	current_velocity = GeoMath::v3(input->vx, input->vy, input->vz);
	
	juk_msg::juk_control_dji_msg output_dji;
	
	if (!set_homepoint_flag)
	{
		calculateVelocity(target.cruising_speed, (target.point_abs - current_point_abs), current_velocity, ctrl_mode);
		
		output_dji.data_x = velocity_need.x;
		output_dji.data_y = velocity_need.y;
		output_dji.data_z = velocity_need.z;
		output_dji.flag = ctrl_flag;
		
		output_dji.course = 0;
	}
	else
	{
		output_dji.data_x = 0;
		output_dji.data_y = 0;
		output_dji.data_z = 0;
	}
	
	juk_msg::juk_position_data_msg position_data;
	
	current_point_home = current_point_abs - homepoint;
	
	position_data.alt = current_point_abs.alt;
	position_data.lat = current_point_abs.lat;
	position_data.lng = current_point_abs.lng;
	
	position_data.x = current_point_home.x;
	position_data.y = current_point_home.y;
	position_data.z = current_point_home.z;

	position_data.course = input->course;
	position_data.dist_to_target = (current_point_abs - target.point_abs).length_xyz();
	
	pub_dji_control.publish(output_dji);
	pub_position_data.publish(position_data);
}


void
NavigationNode::calculateVelocity(double abs_speed, GeoMath::v3 offset, GeoMath::v3 current_velocity,uint8_t ctrl_mode)
{
	ctrl_flag = 5;
	const double max_break_acc = 3.5;
	const double max_force_acc = 0.7;
	double need_abs_speed ;
	
	double max_z_speed = 5;
	double current_speed = current_velocity.length_xyz();
	
	need_abs_speed = std::min(abs_speed, current_speed + max_force_acc);
	double addition_break_time = 0.1;
	double current_distance = offset.length_xyz();
	
	double break_distance = 0;
	
	switch (ctrl_mode)
	{
	case juk_msg::juk_set_target_data_msg::mode_allow_break_distance:
		
		break_distance = ((abs_speed*abs_speed) / (2*max_break_acc));
		if (current_distance < break_distance + addition_break_time*need_abs_speed)
		{
			double max_break_zone_speed = abs_speed;
			double break_zone_speed;
			
			current_distance < 0.1 ? 
				0 :
				(current_distance < 1 ? pow(current_distance, 3) : 2);
			
			if (current_distance < 0.1) break_zone_speed = 0;
			else
			if (current_distance < 1) break_zone_speed = pow(current_distance, 2);
			else
				break_zone_speed = current_distance/3;
				
			if (current_speed > abs_speed && current_distance > 1)
				ctrl_flag = 13;
			need_abs_speed = break_zone_speed;
		}

	}
	
	


	
	velocity_need = offset.normalize_xyz(need_abs_speed);

	if (velocity_need.z > max_z_speed)
		velocity_need = velocity_need * (max_z_speed / velocity_need.z);

	std::cout << "TARGET:\n" << target.point_abs - homepoint << std::endl;
	std::cout << "SPEED:\n" << velocity_need << std::endl;
	
	std_msgs::String msg;

	std::stringstream ss;
	ss << current_point_abs - homepoint;
	msg.data = ss.str();
	
	pub_str.publish(msg);
	//std::cout << current_point_abs << std::endl;
	//std::cout << std::endl;
}

void NavigationNode::set_target_callback(const juk_msg::juk_set_target_data_msg::ConstPtr& target)
{
	this->target.break_mode = target->break_distance_mode;
	this->target.cruising_speed = target->speed;
	this->target.accurancy = target->acc;
	
	switch (target->system)
	{
	case juk_msg::juk_set_target_data_msg::system_absolut:
		this->target.point_abs = GeoMath::v3geo(target->data_x, target->data_y, target->data_z+homepoint.alt);
		break;
		
	case juk_msg::juk_set_target_data_msg::system_home:
		this->target.point_abs = homepoint + GeoMath::v3(target->data_x, target->data_y, target->data_z);
		break;
		
	case juk_msg::juk_set_target_data_msg::system_offset_from_target:
		this->target.point_abs = this->target.point_abs + GeoMath::v3(target->data_x, target->data_y, target->data_z);
		break;	
	}
}