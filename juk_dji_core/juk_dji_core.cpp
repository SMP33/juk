#include <dji_status.hpp> 
#include <dji_vehicle.hpp>
#include "common/dji_linux_helpers.hpp"

#include <juk_msg/juk_dji_gps_msg.h>
#include <juk_msg/juk_dji_device_status_msg.h>
#include <juk_msg/juk_control_dji_msg.h>
#include <juk_msg/juk_dji_camera_control_msg.h>

#include <iostream>
#include <fstream>
#include <LinuxChrono.h>
#include <ros/ros.h>

#include <GeoMath.h>
#include <cstdlib>

#include "gimbal/dji_gimbal.h"


//#define DEBUG_ROS
//
//#define NO_DJI_HARDWARE
//
#define NO_GIMBAL


using namespace std;

Vehicle*   v;
const int F_MODE = 1684;
auto default_ctrlFlag = Control::VERTICAL_VELOCITY | Control::HORIZONTAL_VELOCITY |
     Control::YAW_RATE | Control::STABLE_ENABLE;

DJI::OSDK::Control::CtrlData            default_ctrlData(default_ctrlFlag,0,0,0,0);
DJI::OSDK::Control::CtrlData            current_ctrlData(default_ctrlData);

DJI::OSDK::Telemetry::RCFullRawData  data_RC;
DJI::OSDK::Telemetry::Battery        data_Bat;
DJI::OSDK::Telemetry::GlobalPosition data_GPS;
DJI::OSDK::Telemetry::Velocity       data_Velocity;
DJI::OSDK::Telemetry::SDKInfo        data_Status;
double                               data_Course;

juk_msg::juk_dji_gps_msg msg_GPS;
juk_msg::juk_dji_device_status_msg msg_device_status;

ros::Time last_ctrl_update_time;

int ctrl_flag = juk_msg::juk_control_dji_msg::flag_break;
RotationAngle iAngle, nAngle, cAngle;


int gimbal_motion_time = 20;

common_things::Time t;

struct
{
	float yaw;
	float pitch;
	float roll;
} initAngle,needAngle,currentAngle;

#ifdef NO_DJI_HARDWARE


GeoMath::v3geo sim_pos(3.14 / 4, 3.14 / 3, 100);
const int freq = 5;
GeoMath::v3 vel( 2 / (double)freq,0,0);
#else
const int freq = 30;
#endif // NO_DJI_HARDWARE

int calc_gimbal_speed(int current, int need)
{
	int speed = 800;
	int break_dist = 300;
	int dist = need - current;
	if (abs(dist) < 30) return 0;
	
	if (abs(dist) < break_dist) speed = speed / 2;
	if (abs(dist) < break_dist / 2) speed = speed / 3;
	
	if (dist > 0) return speed;
	return -speed;
}

void ctrl_callback(const juk_msg::juk_control_dji_msg::ConstPtr& msg)
{
	last_ctrl_update_time = ros::Time::now();
	current_ctrlData.x = msg->data_x;
	current_ctrlData.y = msg->data_y;
	current_ctrlData.z = msg->data_z;
	current_ctrlData.yaw = msg->course;	
	ctrl_flag = int(msg->flag);
	
	
}

void gimbal_camera_callback(const juk_msg::juk_dji_camera_control_msg::ConstPtr& msg)
{
	DJI::OSDK::Gimbal::AngleData angleData;
	angleData.mode = 1;
	angleData.yaw = msg->yaw;
	angleData.pitch= msg->pitch;
	angleData.roll = msg->roll;
	angleData.duration = 3;
	
#ifndef NO_GIMBAL

	switch (msg->action)
	{
	case juk_msg::juk_dji_camera_control_msg::take_photo:
		v->camera->shootPhoto();
		break;
	case juk_msg::juk_dji_camera_control_msg::start_video:
		v->camera->videoStart();
		break;
	case juk_msg::juk_dji_camera_control_msg::stop_video:
		v->camera->videoStop();
		break;
	}
#endif // !NO_GIMBAL
//	cout << "~~~~~~~~~~~~~~~~~~~~~~ ";
	//cout << nAngle.yaw << " " << nAngle.pitch << " " << nAngle.roll << endl;
	
	nAngle.yaw = msg->yaw;
	nAngle.pitch = msg->pitch;
	nAngle.roll = msg->roll;

	
	
}

void update_data()
{
#ifndef NO_DJI_HARDWARE
	data_RC =
   v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA>();
	data_Bat =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO>();
	data_Velocity =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_VELOCITY>();
	data_GPS = v->broadcast->getGlobalPosition();

	data_Status =
	  v->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE>();
	Telemetry::Quaternion quat = v->broadcast->getQuaternion();
	double   q2sqr = quat.q2 * quat.q2;
	double   t0    = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
	double   t1    = + 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
	double   t2    = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
	double   t3    = + 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
	double   t4    = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;

	t2 = (t2 > 1.0) ? 1.0 : t2;
	t2 = (t2 < -1.0) ? -1.0 : t2;

	  data_Course = atan2(t1, t0);

	msg_GPS.lat = data_GPS.latitude;
	msg_GPS.lng = data_GPS.longitude;
	msg_GPS.alt = data_GPS.altitude;
	msg_GPS.quality = data_GPS.health;
	msg_GPS.satellites = 0;
	msg_GPS.vx = data_Velocity.data.x;
	msg_GPS.vy = data_Velocity.data.y;
	msg_GPS.vz = data_Velocity.data.z;
	msg_GPS.course = data_Course;
	

	
	
	#else 
	msg_GPS.vx = vel.x / 6370000.0;
	msg_GPS.vy = vel.y / 6370000.0;
	msg_GPS.vz = 0;
	
	sim_pos = sim_pos + vel;
	double r = (3.14 *57)* (double)(-10 + rand() % 20)/200;
	cout << "R: " << r << vel << endl;
	cout<<sim_pos<<endl;
	vel = vel.rotateXY(r*GeoMath::CONST.DEG2RAD);
	msg_GPS.lat = sim_pos.lat*GeoMath::CONST.DEG2RAD;
	msg_GPS.lng = sim_pos.lng*GeoMath::CONST.DEG2RAD;
	msg_GPS.alt = 200;
	msg_GPS.quality = 1;
	msg_GPS.satellites = 0;

	msg_GPS.course = 3.14;
	
	data_RC.lb2.mode = F_MODE;
	#endif
	const long max_mute_duration = 500000000;
	switch (data_RC.lb2.mode)
	{
	case F_MODE:
		{
			auto now = ros::Time::now();
			#ifndef NO_DJI_HARDWARE
			
			if (data_Status.deviceStatus != 2)
			{
				v->obtainCtrlAuthority();
				msg_device_status.changeTime = now; 
			}
			
			if ((now - last_ctrl_update_time).toNSec() > max_mute_duration)
			{
				current_ctrlData = default_ctrlData;
				v->control->emergencyBrake();
			}
				
			else
			{
				//cout << "M: " << ctrl_flag << endl;
				switch (ctrl_flag)
				{
				case 5:
					current_ctrlData.flag = Control::VERTICAL_VELOCITY | Control::HORIZONTAL_VELOCITY |
											Control::YAW_RATE | Control::STABLE_ENABLE;
					v->control->flightCtrl(current_ctrlData);
					break;
				case 7:
					current_ctrlData.flag = Control::VERTICAL_VELOCITY | Control::HORIZONTAL_POSITION |
											Control::YAW_RATE | Control::STABLE_ENABLE;
					v->control->flightCtrl(current_ctrlData);
					break;
				case 13:
					v->control->emergencyBrake();
					break;
				default:
					v->control->emergencyBrake();
					break;
				}			
				
			}
			
			
			
			Telemetry::Gimbal  gibmal;
			#ifndef NO_GIMBAL
			
			gibmal = v->broadcast->getGimbal();
			cAngle.roll  = gibmal.roll * 10 - iAngle.roll;
			cAngle.pitch = gibmal.pitch * 10 - iAngle.pitch;
			cAngle.yaw   = gibmal.yaw * 10;
		
			DJI::OSDK::Gimbal::SpeedData gimbalSpeed;
		
			nAngle.yaw = data_Course * 10*GeoMath::CONST.RAD2DEG;
			
			gimbalSpeed.roll  = calc_gimbal_speed(cAngle.roll, nAngle.roll);
			gimbalSpeed.pitch = calc_gimbal_speed(cAngle.pitch, nAngle.pitch);
			gimbalSpeed.yaw   = calc_gimbal_speed(cAngle.yaw, nAngle.yaw);
			
			gimbalSpeed.gimbal_control_authority = 1;
			gimbalSpeed.disable_fov_zoom = 0;
			gimbalSpeed.ignore_user_stick = 0;
			gimbalSpeed.extend_control_range = 0;
			gimbalSpeed.ignore_aircraft_motion = 0;
			gimbalSpeed.yaw_return_neutral = 0;
			gimbalSpeed.reserved0 = 0;
			gimbalSpeed.reserved1 = 0;
		
			v->gimbal->setSpeed(&gimbalSpeed);
			#endif
			
			#endif
			msg_device_status.authority = juk_msg::juk_dji_device_status_msg::CONTROL_BY_SDK;
			#ifdef DEBUG_ROS
			cout<<"TIME: "<<(now - last_ctrl_update_time).toNSec()<<endl;
			cout << "DATA: " << current_ctrlData.x << " " << current_ctrlData.y << " " << current_ctrlData.z << " " <<ctrl_flag<< endl;
			#endif // !DEBUG_ROS
			
			
		}
		break;
      
	default:
		{	
			#ifndef NO_DJI_HARDWARE
			if (data_Status.deviceStatus == 2)
			{
				v->releaseCtrlAuthority(5000);
				msg_device_status.changeTime = ros::Time::now(); 
			}
			#endif
			
			msg_device_status.authority = juk_msg::juk_dji_device_status_msg::CONTROL_BY_RC;
			msg_device_status.changeTime = ros::Time::now();
		
			break;
		}
	}
	
	msg_device_status.voltage = data_Bat.voltage;
}
int main(int argc, char *argv[])
{
	std::string UserConfig_data = 
"app_id : 1067610\n"
"app_key : b52ab8fdd2d5dd0cce798171cb3581355ab8f64df5d8b4f5d64b87fa0fce0296\n"
"device : /dev/custom/DJI_CONTROLLER\n"
"baudrate : 230400\n";
	std::ofstream UserConfig_file("UserConfig.txt");
	
	UserConfig_file << UserConfig_data;
	
	UserConfig_file.close();
	cout << argv[0] << endl;
	ros::init(argc, argv, "JUK_DJI_CORE_NODE");
	ros::NodeHandle nh;
	last_ctrl_update_time = ros::Time::now();
	ros::Publisher pub_GPS = nh.advertise<juk_msg::juk_dji_gps_msg>("JUK/DJI/GPS", 1);
	ros::Publisher pub_device_status = nh.advertise<juk_msg::juk_dji_device_status_msg>("JUK/DJI/DEVICE_STATUS", 1);
	
	ros::Subscriber sub = nh.subscribe("JUK/CONTROL_DJI", 1, ctrl_callback);
	ros::Subscriber sub_camera = nh.subscribe("JUK/DJI_GIMBAL", 1, gimbal_camera_callback);
	
#ifndef NO_DJI_HARDWARE
	LinuxSetup ls(argc, argv); 
	v = ls.getVehicle();

	auto st=v->broadcast->getStatus();

	//===============�������� �� ��������� ����==========//
	ACK::ErrorCode subscribeStatus;
	subscribeStatus = v->subscribe->verify(5000);

	int                             pkgIndex        = 0;
	int                             freq            = 50;
	DJI::OSDK::Telemetry::TopicName topicList50Hz[] = {
		DJI::OSDK::Telemetry::TOPIC_RC_FULL_RAW_DATA,
		DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO,
		DJI::OSDK::Telemetry::TOPIC_VELOCITY,
		DJI::OSDK::Telemetry::TOPIC_GPS_FUSED,
		DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE
	};

	int  numTopic        = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
	bool enableTimestamp = false;

	bool pkgStatus = v->subscribe->initPackageFromTopicList(
	  pkgIndex,
		numTopic,
		topicList50Hz,
		enableTimestamp,
		freq);

	subscribeStatus = v->subscribe->startPackage(pkgIndex, 50000);
#endif 
	//==========�������� ����==========//
	ros::Rate r(freq);
	
	
	

	auto gibmal = v->broadcast->getGimbal();
	
	iAngle.roll  = gibmal.roll * 10;
	iAngle.pitch = gibmal.pitch * 10;
	iAngle.yaw   = gibmal.yaw * 10;
	
	nAngle.yaw = 0;
	nAngle.pitch = 0;
	nAngle.roll = 0;
	

	while (ros::ok())
	{
		update_data();
		pub_GPS.publish(msg_GPS);
		pub_device_status.publish(msg_device_status);
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}