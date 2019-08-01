

#include "logger.h"

using namespace std;

#define ss <<" "<<
#define ss_(x)  << x <<

ros::Time start_time;

LoggerJuk<juk_msg::juk_dji_gps_msg> gps_logger;

ros::Subscriber sub_dji_gps;
ros::Subscriber sub_dji_device_status;
ros::Subscriber sub_control_dji;

ros::Time dji_gps_time;
ros::Time dji_device_status_time;
ros::Time control_dji_time;

juk_msg::juk_dji_gps_msg juk_dji_gps_data;
juk_msg::juk_dji_device_status_msg  juk_dji_device_status_data;
juk_msg::juk_control_dji_msg juk_control_dji_data;

void callback_dji_gps(const juk_msg::juk_dji_gps_msg::ConstPtr& input)
{
	juk_dji_gps_data = *input;
	gps_logger.data = *input;
	dji_gps_time = ros::Time::now();
	gps_logger.upd_time = ros::Time::now();
}

void callback_dji_device_status(const juk_msg::juk_dji_device_status_msg::ConstPtr& input)
{
	juk_dji_device_status_data = *input;
	dji_device_status_time = ros::Time::now();
}

void callback_control_dji(const juk_msg::juk_control_dji_msg::ConstPtr& input)
{
	juk_control_dji_data = *input;
	control_dji_time  = ros::Time::now();
}

void log()
{
	auto now = ros::Time::now();
	stringstream result;
	string h1 = "\n* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n";
	
	stringstream juk_dji_gps_str; 
	juk_dji_gps_str.precision(8);
	juk_dji_gps_str << "JUK_DJI_GPS\n" 
		ss_("last upd:") dji_gps_time
		ss_("\nlat:") juk_dji_gps_data.lat ss_("; lng:") juk_dji_gps_data.lng ss_("; alt:") juk_dji_gps_data.alt ss_("; vx:") juk_dji_gps_data.vx ss_("; vy:") juk_dji_gps_data.vy ss_("; vz:") juk_dji_gps_data.vz ss_("; course:") juk_dji_gps_data.course ss_("; Q:") juk_dji_gps_data.quality << endl;
	stringstream juk_dji_device_status_str;
	stringstream juk_control_dji_str;
	
	juk_control_dji_str << "JUK_CONTROL_DJI\n" 
		ss_("last upd:") control_dji_time
		ss_("\nflag:") juk_control_dji_data.flag ss_("; x:") juk_control_dji_data.data_x ss_("; y:") juk_control_dji_data.data_y ss_("; z:") juk_control_dji_data.data_z ss_("; course:") juk_control_dji_data.course << endl;
	
	result << h1 << "START TIME:" ss start_time ss "TIME:" ss now <<endl << juk_dji_gps_str.str() << juk_control_dji_str.str();
	
	cout << result.str();
	
}

int main(int argc, char *argv[])
{
	
	
	ros::init(argc, argv, "JUK_LOGGER");
	ros::NodeHandle nh; 
	
	//sub_dji_gps = nh.subscribe("JUK/DJI/GPS", 1,callback_dji_gps);
	//sub_dji_device_status = nh.subscribe("JUK/DJI/DEVICE_STATUS", 1, callback_dji_device_status);
	//sub_control_dji = nh.subscribe("JUK/CONTROL_DJI", 1, callback_control_dji);
	
	start_time = ros::Time::now();
	
	
	//gps_logger.sub = nh.subscribe("JUK/DJI/GPS", 1, callback_dji_gps);
	
	ros::Rate r(5);
	
	JukLogger<juk_msg::juk_dji_gps_msg> gps_log(&nh, "JUK/DJI/GPS");
	while (ros::ok())
	{
		cout << gps_log.data.lat << endl;
		cout << "123" << endl;
		//cout << gps_logger.data.lat << endl;
		//log();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}