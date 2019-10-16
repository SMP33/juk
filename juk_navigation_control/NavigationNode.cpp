
#include "NavigationNode.h"

#define CLEAR(rows)               printf("\033[%02dA\033[0J",rows+1)

void 
NavigationNode::init_handlers()
{

	state_handlers[STATES::IDLE] = [this]()->CtrlStatus
	{
		CtrlStatus ctrl;
		juk_msg::juk_control_dji_msg msg;
		msg.flag = 5;
		
		ctrl.msg = msg;
		ctrl.stable_now = true;
		return ctrl ;
	};
	
	state_handlers[STATES::FLY_SIMPLE] = [this]()->CtrlStatus
	{
		this->SUB_STATE = 0;
		CtrlStatus ctrl;
		GeoMath::v3 position_offset = current_target.point_abs - current_point_abs;
			
		ctrl.msg = calculateControl(position_offset,
			current_velocity, 
			current_target.course,
			position_data.course,
			current_target.cruising_speed,
			juk_msg::juk_set_target_data_msg::mode_allow_break_distance);
		
		ctrl.stable_now = (position_data.dist_to_target <= target.accurancy);
		
		return ctrl;
	};
	
	state_handlers[STATES::FLY_SAFE] = [this]()->CtrlStatus
	{
		CtrlStatus ctrl;
		GeoMath::v3 position_offset = current_target.point_abs - current_point_abs;
	
		if (!SUB_STATE)
			SUB_STATE = SUB_STATES::FLY_SAFE_UP;		
		
		switch (SUB_STATE)
		{
		case SUB_STATES::FLY_SAFE_UP :
						
			if (current_point_abs.alt - homepoint.alt < params.args["safe_alt"])
			{
				sub_target = current_target;
				sub_target.point_abs = GeoMath::v3geo(current_point_abs.lat, current_point_abs.lng, params.args["safe_alt"] + homepoint.alt);
				GeoMath::v3 sub_position_offset =  sub_target.point_abs - current_point_abs;
				ctrl.msg = calculateControl(sub_position_offset,
					current_velocity,
					current_target.course,
					position_data.course,
					current_target.cruising_speed,
					juk_msg::juk_set_target_data_msg::mode_not_break_distance);
				//calculateVelocity(current_target.cruising_speed, sub_position_offset, current_velocity, juk_msg::juk_set_target_data_msg::mode_not_break_distance) ;
			}
			else
			{
				sub_target = current_target;
				sub_target.point_abs.alt = params.args["safe_alt"] + homepoint.alt;
				GeoMath::v3 sub_position_offset =  sub_target.point_abs - current_point_abs;
				ctrl.msg = calculateControl(sub_position_offset,
					current_velocity,
					current_target.course,
					position_data.course,
					current_target.cruising_speed,
					juk_msg::juk_set_target_data_msg::mode_not_break_distance);
				//calculateVelocity(current_target.cruising_speed, sub_position_offset, current_velocity, ctrl_mode) ;
			}
				
		
			if (position_offset.length_xy() < 2)
			{
				SUB_STATE = SUB_STATES::FLY_SAFE_CENTER;
		
				GeoMath::v3 position_offset = current_target.point_abs - current_point_abs;
		
				ctrl.msg = calculateControl(position_offset,
					current_velocity,
					current_target.course,
					position_data.course,
					current_target.cruising_speed,
					ctrl_mode);
			}

			break ;
		
		case SUB_STATES::FLY_SAFE_CENTER :
			if (position_offset.length_xy() > 5)
			{
				SUB_STATE = SUB_STATES::FLY_SAFE_CENTER;
				break ;
			}
			
			if (position_offset.length_xy() > 1)
			{
			
				ctrl.msg = calculateControl( GeoMath::v3(position_offset.x, position_offset.y, 0),
					current_velocity,
					current_target.course,
					position_data.course,
					current_target.cruising_speed,
					ctrl_mode);
			}
			else
			{
				ctrl.msg = calculateControl(position_offset,
					current_velocity,
					current_target.course,
					position_data.course,
					current_target.cruising_speed,
					ctrl_mode);
			}
				
			if (position_offset.length_xyz() < current_target.accurancy)
			{
				STATE = STATES::FLY_SIMPLE;
				SUB_STATE = 0;					
			}
				
			break ;
			
		default :
			SUB_STATE = 0;
			break ;
		}
		;
		
		ctrl.stable_now = false;
		
		return ctrl ;
	};
	
	state_handlers[STATES::LANDING_SIMPLE] = [this]()->CtrlStatus
	{
		CtrlStatus ctrl;
		
		if (!SUB_STATE)
			SUB_STATE = SUB_STATES::LANDING_SIMPLE_FLY;
		
		GeoMath::v3 position_offset = current_target.point_abs - current_point_abs;	
		
		switch (SUB_STATE)
		{
		case SUB_STATES::LANDING_SIMPLE_FLY :
			
			if (position_offset.length_xyz() < current_target.accurancy)
			{
				STATE = STATES::LANDING_SIMPLE;
				SUB_STATE = SUB_STATES::LANDING_SIMPLE_LAND;					
			}
			
			ctrl.msg = calculateControl(position_offset,
				current_velocity, 
				current_target.course,
				position_data.course,
				current_target.cruising_speed,
				juk_msg::juk_set_target_data_msg::mode_allow_break_distance);
			
			break ;
				
		case SUB_STATES::LANDING_SIMPLE_LAND :
				
			if (this->flight_status < 1)
				set_homepoint_flag = true;
			
			ctrl.msg = calculateControl(GeoMath::v3(0, 0, -100),
				current_velocity, 
				0,
				0,
				0.6,
				juk_msg::juk_set_target_data_msg::mode_not_break_distance);
			
			break ;
				
		default :
			break ;
		}
		
		ctrl.stable_now = false;
		
		return ctrl;
	};
	
	state_handlers[STATES::LANDING_ARUCO] = [this]()->CtrlStatus
	{ 
	
		CtrlStatus ctrl;
		
		const long int aruco_timeout = 1000000000;
		if (!SUB_STATE || (ros::Time::now() - aruco_land.uptime).toNSec() >= aruco_timeout)
			SUB_STATE = SUB_STATES::LANDING_ARUCO_FLY;
		else
			if ((ros::Time::now() - aruco_land.uptime).toNSec() < aruco_timeout && SUB_STATE == SUB_STATES::LANDING_ARUCO_FLY)
			SUB_STATE = SUB_STATES::LANDING_ARUCO_LAND;
				
		switch (SUB_STATE)
		{
		case SUB_STATES::LANDING_ARUCO_FLY :
			
			{
				GeoMath::v3 position_offset = current_target.point_abs - current_point_abs;
				ctrl.msg = calculateControl(position_offset, current_velocity, current_target.course, position_data.course, current_target.cruising_speed, juk_msg::juk_set_target_data_msg::mode_allow_break_distance);
			}
			break;

		case SUB_STATES::LANDING_ARUCO_LAND :
			{		
				sub_target.cruising_speed = 0.7;
					
				if (flight_status < 1)
					set_homepoint_flag = true;
			
				//std::cout << aruco_land.offset << std::endl;
			
				double max_dist = abs(aruco_land.offset.z) / 7 + 0.1;
				double current_dist = aruco_land.offset.length_xy();
			
				if (current_dist > max_dist)
				{
					target.course = aruco_land.course;
					target_precision.course = aruco_land.course;
					current_target.course = aruco_land.course;
					sub_target.course = aruco_land.course;
				
				
					GeoMath::v2 cC(cos(current_target.course*GeoMath::CONST.DEG2RAD), sin(current_target.course*GeoMath::CONST.DEG2RAD));
					GeoMath::v2 cN(cos(position_data.course*GeoMath::CONST.DEG2RAD), sin(position_data.course*GeoMath::CONST.DEG2RAD));

					ctrl.msg.course = 0;
					//msg.course = cC.angle_xy(cN)*GeoMath::CONST.RAD2DEG / 2;
				
					ctrl.msg.data_x = aruco_land.offset.x;
					ctrl.msg.data_y = -aruco_land.offset.y;
				
					ctrl.msg.flag = 8;
				}
				else
				{
					target.course = aruco_land.course;
					target_precision.course = aruco_land.course;
					current_target.course = aruco_land.course;
					sub_target.course = aruco_land.course;
				
				
				
					GeoMath::v2 cC(cos(current_target.course*GeoMath::CONST.DEG2RAD), sin(current_target.course*GeoMath::CONST.DEG2RAD));
					GeoMath::v2 cN(cos(position_data.course*GeoMath::CONST.DEG2RAD), sin(position_data.course*GeoMath::CONST.DEG2RAD));

				
					ctrl.msg.data_x = aruco_land.offset.x;
					ctrl.msg.data_y = -aruco_land.offset.y;
					ctrl.msg.course = cC.angle_xy(cN)*GeoMath::CONST.RAD2DEG / 2;
				
					if (abs(ctrl.msg.course) < 2)
					{
					
						double dist_coeff = (max_dist - current_dist) / max_dist;
					
						if (dist_coeff < 0)
							dist_coeff = 0;
					
						if (aruco_land.offset.z > 2.0)
							ctrl.msg.data_z = -0.5*dist_coeff;
						else
							ctrl.msg.data_z = -0.1;
					}
					else
					{
			
						ctrl.msg.data_x = 0;
						ctrl.msg.data_y = 0;
						ctrl.msg.data_z = 0;
					}
				
					ctrl.msg.flag = 8;
				}
			}
			break ;
		default :
			
			break ;
		}
		
		ctrl.stable_now = false;
		
		return ctrl;
	};
}

NavigationNode::NavigationNode(int argc, char** argv) 
{
	std::cout.flags(std::ios::fixed);
	std::cout.precision(2);
	
	params.args["safe_alt"] = 20;
	params.args["enable_emlid"] = 0;
	params.args["gear_height"] = 2;
	params.args["aruco_marker_id"] = 10;
	params.args["aruco_marker_size"] = 180;
	
	usleep(5000000);
	
	params.parse(argc, argv);
	std::cout << c(32, "@Parameters JUK_NAVIGATION_NODE: ") << std::endl;
	for (auto arg : params.args)
	{
		std::cout << c(32, "~~") << arg.first << ": " << c(32, arg.second) << std::endl;
	}
	
	node_start_time = ros::Time::now();
	aruco_land.uptime = node_start_time;
	home_uptime = node_start_time;
	last_telemetry = node_start_time;
	
	pub_dji_control = nh.advertise<juk_msg::juk_control_dji_msg>("JUK/CONTROL_DJI", 1);
	pub_position_data = nh.advertise<juk_msg::juk_position_data_msg>("JUK/POSITION_DATA", 1);
	pub_aruco_action = nh.advertise<juk_msg::juk_aruco_module_action>("JUK/ARUCO/ACTION", 1);
	
	target.cruising_speed = 1;
	target.accurancy = 0.3;
	target.course = 0;
	yaw_rate = 0;
	precision_pos_quality = 6;
	ctrl_mode = juk_msg::juk_set_target_data_msg::mode_allow_break_distance;

	sub_aruco_data = nh.subscribe("JUK/ARUCO/DATA", 1, &NavigationNode::aruco_callback, this);
	sub_dji_gps = nh.subscribe("JUK/DJI/GPS", 1, &NavigationNode::gps_callback, this);
	sub_set_target = nh.subscribe("JUK/TARGET", 1, &NavigationNode::set_target_callback, this);
	sub_precision_gps = nh.subscribe("REACH_EMLID_DATA", 1, &NavigationNode::precision_gps_callback, this);
	sub_action_process = nh.subscribe("JUK/NAVIGATION_ACTIONS", 1, &NavigationNode::action_process_callback, this);
	
	stable_now = false;
	stable_last = false;
	
	
	juk_msg::juk_aruco_module_action aruco_msg;
	aruco_msg.id = 10;
	aruco_msg.size = 180;
	aruco_msg.action = 1;
	
	set_homepoint_flag = true;
		
	pub_aruco_action.publish(aruco_msg);	
	init_handlers();
	
	for (int i = 0; i < telem_heigth + 3; i++)
	{
		std::cout << std::endl;
	}
}

juk_msg::juk_control_dji_msg 
NavigationNode::calculateControl(GeoMath::v3 offset, GeoMath::v3 current_velocity, double course_need, double course_current, double abs_speed, uint8_t ctrl_mode)
{
	juk_msg::juk_control_dji_msg ans;
	ans.flag = 5;
	
	const double max_break_acc = 5;
	const double max_force_acc = 0.7;
	double need_abs_speed;
	
	double max_z_speed = 5;
	
	
	need_abs_speed = std::min(abs_speed, current_velocity.length_xyz() + max_force_acc);
	double addition_break_time = 0.1;
	double current_distance = offset.length_xyz();
	
	double break_distance = 0;
	
	switch (ctrl_mode)
	{
	case juk_msg::juk_set_target_data_msg::mode_allow_break_distance :
		
		break_distance = ((abs_speed*abs_speed) / (2*max_break_acc));
		
		if (current_distance < break_distance + addition_break_time*need_abs_speed + 1.5)
		{
			ans.flag = 7;
		}
		break ;
		
	case 88:
		ans.flag = 8;
		break;
	}
	
	GeoMath::v2 cC(cos(course_current*GeoMath::CONST.DEG2RAD), sin(course_current*GeoMath::CONST.DEG2RAD));
	GeoMath::v2 cN(cos(course_need*GeoMath::CONST.DEG2RAD), sin(course_need*GeoMath::CONST.DEG2RAD));

	ans.course = -cC.angle_xy(cN)*GeoMath::CONST.RAD2DEG / 2;
		
	GeoMath::v3 velocity_need = offset.normalize_xyz(need_abs_speed);

	if (abs(ans.course) < 3)
	{
		if (velocity_need.z > max_z_speed)
			velocity_need = velocity_need * (max_z_speed / velocity_need.z);
	
		if (ans.flag == 7)
		{
			velocity_need.x = offset.x;
			velocity_need.y = offset.y;
			velocity_need.z = offset.z / 2; 
		}
	}
	else
	{
		ans.flag = 5;
		velocity_need.x = 0;
		velocity_need.y = 0;
		velocity_need.z = 0;
	}
	
	ans.data_x = velocity_need.x; 
	ans.data_y = velocity_need.y;
	ans.data_z = velocity_need.z;
	
	return ans ;
}

//void
//NavigationNode::calculateVelocity(double abs_speed, GeoMath::v3 offset, GeoMath::v3 current_velocity, uint8_t ctrl_mode)
//{
//	ctrl_flag = 5;
//	const double max_break_acc = 5;
//	const double max_force_acc = 0.7;
//	double need_abs_speed;
//	
//	double max_z_speed = 5;
//	double current_speed = current_velocity.length_xyz();
//	
//	need_abs_speed = std::min(abs_speed, current_speed + max_force_acc);
//	double addition_break_time = 0.1;
//	double current_distance = offset.length_xyz();
//	
//	double break_distance = 0;
//	
//	switch (ctrl_mode)
//	{
//	case juk_msg::juk_set_target_data_msg::mode_allow_break_distance :
//		
//		break_distance = ((abs_speed*abs_speed) / (2*max_break_acc));
//		
//		if (current_distance < break_distance + addition_break_time*need_abs_speed + 1.5)
//		{
//			ctrl_flag = 7;
//		}
//		break ;
//	}
//	
//	GeoMath::v2 cC(cos(position_data.course*GeoMath::CONST.DEG2RAD), sin(position_data.course*GeoMath::CONST.DEG2RAD));
//	GeoMath::v2 cN(cos(target.course*GeoMath::CONST.DEG2RAD), sin(target.course*GeoMath::CONST.DEG2RAD));
//
//	yaw_rate = -cC.angle_xy(cN)*GeoMath::CONST.RAD2DEG / 2;
//		
//	velocity_need = offset.normalize_xyz(need_abs_speed);
//	//std::cout.flags(std::ios::fixed);
//	//std::cout << offset << std::endl;
//	if(abs(yaw_rate) < 3)
//	{
//		if (velocity_need.z > max_z_speed)
//			velocity_need = velocity_need * (max_z_speed / velocity_need.z);
//	
//		if (ctrl_flag == 7)
//		{
//			velocity_need.x = offset.x;
//			velocity_need.y = offset.y;
//			velocity_need.z = offset.z / 2; 
//		}
//	}
//	else
//	{
//		ctrl_flag = 5;
//		velocity_need.x = 0;
//		velocity_need.y = 0;
//		velocity_need.z = 0;
//	}
//
//}

void NavigationNode::aruco_callback(const juk_msg::juk_aruco_module_data::ConstPtr& input)
{
	aruco_land.offset = GeoMath::v3(-input->x / 100, -input->y / 100, -input->z / 100);
	aruco_land.course = position_data.course +  input->course * GeoMath::CONST.RAD2DEG;
	aruco_land.uptime = ros::Time::now();
}

void
NavigationNode::action_process_callback(const juk_msg::juk_navigation_actions_msg::ConstPtr& input)
{
	int action = input->action;
	
	switch (action)
	{
		
	case ACTIONS::SET_HOMEPOINT :
		set_homepoint_flag = true;
		break ;
	default :
		break ;
	}
}

void
NavigationNode::gps_callback(const juk_msg::juk_dji_gps_msg::ConstPtr& input)
{
	auto now = ros::Time::now();
	bool allow_emlid = (params.args["enable_emlid"]&&(precision_pos_quality == 1 || precision_pos_quality == 2)) || !params.args["enable_emlid"];
	
	flight_status = input->flight_status;
	
	if ((set_homepoint_flag&&(now - node_start_time).toNSec() > 5000000000)
		&&allow_emlid)
	{
		target.course = input->course*GeoMath::CONST.RAD2DEG;
		homepoint_course = input->course;
		homepoint = GeoMath::v3geo(input->lat*GeoMath::CONST.RAD2DEG, input->lng*GeoMath::CONST.RAD2DEG, input->alt);
		homepoint_precision = GeoMath::v3geo(precision_position.lat, precision_position.lng, input->alt);
		target.point_abs = homepoint;
		target_precision.point_abs = homepoint_precision;
		
		set_homepoint_flag = false;
		
		STATE = STATES::IDLE;
		SUB_STATE = SUB_STATES::BASIC_STATE;
		
		home_uptime = ros::Time::now();
		//std::cout << "HOMEPOINT SET" << std::endl;
	}
	
	
	if (allow_emlid && params.args["enable_emlid"]&&(now - precision_pos_uptime).toNSec() < 1000000000)
	{
		//std::cout << "Emlid" << std::endl;
		GPS_STATE = GPS_STATES::RTK;
		current_point_abs = precision_position;
		current_target = target_precision;
	}
	else
	{
		//std::cout << "A3" << std::endl;
		GPS_STATE = GPS_STATES::BASIC_GPS;
		current_point_abs = GeoMath::v3geo(input->lat*GeoMath::CONST.RAD2DEG, input->lng*GeoMath::CONST.RAD2DEG, input->alt);
		current_target = target;
	}	
	
	current_velocity = GeoMath::v3(input->vx, input->vy, input->vz);
	
	current_point_home = current_point_abs - homepoint_precision;
	
	position_data.alt = current_point_abs.alt;
	position_data.lat = current_point_abs.lat;
	position_data.lng = current_point_abs.lng;
	
	position_data.x = current_point_home.x;
	position_data.y = current_point_home.y;
	position_data.z = current_point_home.z;

	position_data.course = input->course*GeoMath::CONST.RAD2DEG;
	
	GeoMath::v3 position_offset = current_target.point_abs - current_point_abs;
	
	if (!set_homepoint_flag)
	{
		CtrlStatus ctrl;
		
		if (state_handlers.find(STATE) != state_handlers.end())
			ctrl = state_handlers[STATE]();
		else
		{
			ctrl.msg.data_x = 0;
			ctrl.msg.data_y = 0;
			ctrl.msg.data_z = 0;
			ctrl.msg.flag = 5;
			ctrl.msg.course = 0;
			
			ctrl.stable_now = false;
		}
		
		output_dji = ctrl.msg;
		stable_now = ctrl.stable_now;
	}
	else
	{
		stable_now = true;
		output_dji.data_x = 0;
		output_dji.data_y = 0;
		output_dji.data_z = 0;
	}
	
	position_data.dist_to_target = (current_point_abs - target.point_abs).length_xyz();
	
	GeoMath::v3 pos_from_home = (GeoMath::v3geo(position_data.lat, position_data.lng, position_data.alt) - homepoint).rotateXY(homepoint_course);
	
	position_data.x = pos_from_home.x;
	position_data.y = pos_from_home.y;
	position_data.z = pos_from_home.z;
	
	if (stable_now)
	{
		if (!stable_last)
		{
			stable_start = ros::Time::now();
		}
		stable_time = (ros::Time::now() - stable_start).sec;
		position_data.stable_time = stable_time;
	}
	else
	{
		stable_time = 0;
		position_data.stable_time = 0;
	}
	
	
	
	stable_last = stable_now;
	
	if ((ros::Time::now() - node_start_time).sec > 5)
	{
		pub_dji_control.publish(output_dji);
		pub_position_data.publish(position_data);
		print_telemetry();
	}
}


void 
NavigationNode::precision_gps_callback(const juk_msg::reach_msg::ConstPtr& in)
{
	precision_pos_uptime = ros::Time::now();
	precision_pos_quality = in->quality;
	
	precision_position.lat = in->lat;
	precision_position.lng = in->lng;
	precision_position.alt = current_point_abs.alt;
}

void NavigationNode::set_target_callback(const juk_msg::juk_set_target_data_msg::ConstPtr& target)
{
	STATE = (int)target->fly_mode;
	SUB_STATE = 0;
	//std::cout << "STATE: " << STATE << std::endl;
	this->target.break_mode = target->break_distance_mode;
		
	this->target.cruising_speed = target->speed;
	this->target.accurancy = target->acc;
	
	
	this->target_precision.cruising_speed = target->speed;
	this->target_precision.accurancy = target->acc;
	
	

	
	switch (target->system)
	{
	case juk_msg::juk_set_target_data_msg::system_absolut :
		this->target.point_abs = GeoMath::v3geo(target->data_x, target->data_y, target->data_z + homepoint.alt);
		this->target_precision.point_abs = GeoMath::v3geo(target->data_x, target->data_y, target->data_z + homepoint.alt);
		break ;
		
	case juk_msg::juk_set_target_data_msg::system_home :
		this->target.point_abs = homepoint + GeoMath::v3(target->data_x, target->data_y, target->data_z);
		this->target_precision.point_abs = homepoint_precision + GeoMath::v3(target->data_x, target->data_y, target->data_z);
		break ;
		
	case juk_msg::juk_set_target_data_msg::system_offset_from_target :
		this->target.point_abs = this->target.point_abs + GeoMath::v3(target->data_x, target->data_y, target->data_z);
		this->target_precision.point_abs = this->target_precision.point_abs + GeoMath::v3(target->data_x, target->data_y, target->data_z);
		break ;	
		
	case juk_msg::juk_set_target_data_msg::system_offset_from_here :
		this->target.point_abs = GeoMath::v3geo(this->position_data.lat, this->position_data.lng, this->position_data.alt) + GeoMath::v3(target->data_x, target->data_y, target->data_z).rotateXY(this->position_data.course*GeoMath::CONST.DEG2RAD);
		this->target_precision.point_abs = GeoMath::v3geo(this->position_data.lat, this->position_data.lng, this->position_data.alt) + GeoMath::v3(target->data_x, target->data_y, target->data_z).rotateXY(this->position_data.course*GeoMath::CONST.DEG2RAD);
		break ;	
	}
	
	switch (target->course_mode)
	{
	case juk_msg::juk_set_target_data_msg::course_abs:		
		{
			this->target.course = target->course;
			this->target_precision.course = target->course;
		}
		break;
		
	case juk_msg::juk_set_target_data_msg::course_add:		
		{
			double rotate_rad = target->course*GeoMath::CONST.DEG2RAD;
			
			this->position_data.course = GeoMath::CONST.RAD2DEG*(GeoMath::v2(cos(this->position_data.course*GeoMath::CONST.DEG2RAD), sin(this->position_data.course*GeoMath::CONST.DEG2RAD)).rotateXY(rotate_rad)).angle_xy(GeoMath::v2(1, 0));
		}
		break;
		
	case juk_msg::juk_set_target_data_msg::course_from_move_dir:		
		{
			double course_rad = target->course*GeoMath::CONST.DEG2RAD;
			
			double course_deg = (this->target.point_abs - GeoMath::v3geo(this->position_data.lat, this->position_data.lng, this->position_data.alt)).rotateXY(course_rad).angle_xy(GeoMath::v3(1, 0, 0))*GeoMath::CONST.RAD2DEG;
			
			if (course_deg != course_deg)
			{
				course_deg = this->position_data.course;
			}
			
			this->target.course = course_deg;
			this->target_precision.course = course_deg;
		}
		break;
		
	default:
		break;
	}
	
	//std::cout << "TARGET:\n" << this->target.point_abs - homepoint << std::endl;
}
void NavigationNode::print_telemetry()
{
	auto now_time = ros::Time::now();
	if ((now_time - last_telemetry).nsec >  1000000000.0 * 0.15)
	{
		CLEAR(telem_heigth - 1);
				
		std::cout << "STATE:\n\t" << state_map[STATE] << std::endl;
		std::cout << "SUB STATE:\n\t" << sub_state_map[SUB_STATE] << std::endl;
		std::cout << "GPS STATE:\n\t" << gps_state_map[GPS_STATE] << std::endl;
		std::cout << "POSITION:\n\t" << GeoMath::v3(position_data.x, position_data.y, position_data.z) << " c: " << position_data.course << std::endl;
		std::cout << "TARGET:\n\t" << (this->target.point_abs - homepoint) << " c: " << this->target.course << std::endl;
		std::cout << "STABLE TIME:\n\t" << stable_time << std::endl;
		
		if ((now_time - aruco_land.uptime).sec < 10)
		{
			std::cout << "ARUCO POSITION:\n\t" << aruco_land.offset << std::endl;
		}
		else
		{
			std::cout << "ARUCO POSITION:\n\t" << "x: ~ \t y: ~ \t z: ~" << std::endl;
		}
		
		
		last_telemetry = now_time;
	}
}