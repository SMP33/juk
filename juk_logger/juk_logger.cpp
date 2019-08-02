

#include "logger.h"

#include <juk_msg/juk_dji_gps_msg.h>
#include <juk_msg/juk_dji_device_status_msg.h>
#include <juk_msg/juk_control_dji_msg.h>
#include <juk_msg/juk_position_data_msg.h>
#include <juk_msg/juk_set_target_data_msg.h>
#include <juk_msg/reach_msg.h>

#define pr(x,y) <<  x <<": "<<y<<endl

using namespace std;

int main(int argc, char *argv[])
{
	
	ros::init(argc, argv, "JUK_LOGGER");
	ros::NodeHandle nh; 	
	
	ros::Rate r(1);
	ros::Time start_time = ros::Time::now();
	cout << start_time << endl;
	
	SimpleSub<juk_msg::juk_dji_gps_msg> gps(&nh, "JUK/DJI/GPS", start_time);
	SimpleSub<juk_msg::juk_dji_device_status_msg> device_status(&nh, "JUK/DJI/DEVICE_STATUS", start_time);
	
	SimpleSub<juk_msg::juk_control_dji_msg> control_dji(&nh, "JUK/CONTROL_DJI", start_time);
	SimpleSub<juk_msg::juk_position_data_msg> position_data(&nh, "JUK/POSITION_DATA", start_time);
	
	SimpleSub<juk_msg::juk_set_target_data_msg> target(&nh, "JUK/TARGET", start_time);
	
	SimpleSub<juk_msg::reach_msg> reach(&nh, "REACH_EMLID_DATA", start_time);
	
	
	while (ros::ok())
	{
		stringstream str;
		
		gps.reset_str();
		gps.add_str("lat", gps.data.lat);
		gps.add_str("lng", gps.data.lng);
		gps.add_str("alt", gps.data.alt);
		gps.add_str("vx", gps.data.vx);
		gps.add_str("vy", gps.data.vy);
		gps.add_str("vz", gps.data.vz);
		gps.add_str("course", gps.data.course);
		gps.add_str("sats", int(gps.data.satellites));
		gps.add_str("quality", int(gps.data.quality));
		
		device_status.reset_str();
		device_status.add_str("authority", int(device_status.data.authority));
		device_status.add_str("voltage", device_status.data.voltage);
		
		control_dji.reset_str();
		control_dji.add_str("flag", int(control_dji.data.flag));
		control_dji.add_str("data_x", control_dji.data.data_x);
		control_dji.add_str("data_y", control_dji.data.data_y);
		control_dji.add_str("data_z", control_dji.data.data_z);
		control_dji.add_str("course", control_dji.data.course);
		
		position_data.reset_str();
		position_data.add_str("lat", position_data.data.lat);
		position_data.add_str("lng", position_data.data.lng);
		position_data.add_str("alt", position_data.data.alt);
		position_data.add_str("x", position_data.data.x);
		position_data.add_str("y", position_data.data.y);
		position_data.add_str("z", position_data.data.z);
		position_data.add_str("course", position_data.data.course);
		position_data.add_str("dist_to_target", position_data.data.dist_to_target);
		position_data.add_str("stable_time", position_data.data.stable_time);
		
		reach.reset_str();
		reach.add_str("lat", reach.data.lat);
		reach.add_str("lng", reach.data.lng);
		reach.add_str("alt", reach.data.alt);
		reach.add_str("quality", int(reach.data.quality));
		reach.add_str("data_Y", reach.data.time_Y);
		reach.add_str("data_M", reach.data.time_M);
		reach.add_str("data_D", reach.data.time_D);
		reach.add_str("time_h", reach.data.time_h);
		reach.add_str("time_m", reach.data.time_m);
		reach.add_str("time_s", reach.data.time_s);
		
		str.flags(std::ios::fixed);
		str.precision(10);
		
		str << "~ ~ ~ ~ ~"  << endl;
		str << "TIME:[" << (ros::Time::now() - start_time) << "]" << endl;
		
		str << gps.get_full_str();
		gps.clean_upd();
		
		str << device_status.get_full_str();
		device_status.clean_upd();
		
		str << position_data.get_full_str();
		position_data.clean_upd();
		
		str << control_dji.get_full_str();
		control_dji.clean_upd();
		
		str << reach.get_full_str();
		reach.clean_upd();
		
		cout << str.str();
		
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}