#include <fstream>

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
	
	ofstream log_file;
	log_file.open("last.log");
	
	SimpleServer host(20045);
	
	unsigned long id=0;
	
	while (ros::ok())
	{
		stringstream str;
		
		gps.reset_str();
		gps.add_str("lat", gps.data.lat, true);
		gps.add_str("lng", gps.data.lng, true);
		gps.add_str("alt", gps.data.alt, true);
		gps.add_str("vx", gps.data.vx, true);
		gps.add_str("vy", gps.data.vy, true);
		gps.add_str("vz", gps.data.vz, true);
		gps.add_str("course", gps.data.course, true);
		gps.add_str("sats", int(gps.data.satellites), true);
		gps.add_str("quality", int(gps.data.quality), true);
		gps.add_str("flight_status", int(gps.data.flight_status), false);
		
		device_status.reset_str();
		device_status.add_str("authority", int(device_status.data.authority), true);
		device_status.add_str("voltage", device_status.data.voltage, false);
		
		control_dji.reset_str();
		control_dji.add_str("flag", int(control_dji.data.flag), true);
		control_dji.add_str("data_x", control_dji.data.data_x, true);
		control_dji.add_str("data_y", control_dji.data.data_y, true);
		control_dji.add_str("data_z", control_dji.data.data_z, true);
		control_dji.add_str("course", control_dji.data.course, false);
		
		position_data.reset_str();
		position_data.add_str("lat", position_data.data.lat, true);
		position_data.add_str("lng", position_data.data.lng, true);
		position_data.add_str("alt", position_data.data.alt, true);
		position_data.add_str("x", position_data.data.x, true);
		position_data.add_str("y", position_data.data.y, true);
		position_data.add_str("z", position_data.data.z, true);
		position_data.add_str("course", position_data.data.course, true);
		position_data.add_str("dist_to_target", position_data.data.dist_to_target, true);
		position_data.add_str("stable_time", position_data.data.stable_time, false);
		
		reach.reset_str();
		reach.add_str("lat", reach.data.lat, true);
		reach.add_str("lng", reach.data.lng, true);
		reach.add_str("alt", reach.data.alt, true);
		reach.add_str("quality", int(reach.data.quality), true);
		reach.add_str("data_Y", reach.data.time_Y, true);
		reach.add_str("data_M", reach.data.time_M, true);
		reach.add_str("data_D", reach.data.time_D, true);
		reach.add_str("time_h", reach.data.time_h, true);
		reach.add_str("time_m", reach.data.time_m, true);
		reach.add_str("time_s", reach.data.time_s, false);
		
		str.flags(std::ios::fixed);
		str.precision(10);
		
		str << "MESSAGE_BEGIN" << endl;
		
		str << "{" << endl;
		str << "\"HEADER\": {" << endl;
		str << "\"ID\": " << id <<","<< endl;
		str << "\"TIME\": " << (ros::Time::now() - start_time) << endl;
		str << "}," << endl;
		str << "\"DATA\":{" << endl;
		
		str << gps.get_full_str() << "," << endl;
		gps.clean_upd();
		
		str << device_status.get_full_str() << "," << endl;
		device_status.clean_upd();
		
		str << position_data.get_full_str() << "," << endl;
		position_data.clean_upd();
		
		str << control_dji.get_full_str() << "," << endl;
		control_dji.clean_upd();
		
		str << reach.get_full_str();
		reach.clean_upd();
		
		str << "}\n}" << endl;
		
		str << "MESSAGE_END" << endl<<endl;
		
		//cout << str.str() << endl;;
		log_file<<str.str();
		host.set_response(str.str());
		//cout << "ID: "<<id << endl;
		ros::spinOnce();
		r.sleep();
		id++;
	}
	return 0;
}