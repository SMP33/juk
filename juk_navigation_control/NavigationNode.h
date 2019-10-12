#pragma once
#ifndef __NAVIGATION_NODE__
#define __NAVIGATION_NODE__
				 
#include <ros/ros.h>

#include <tuple>
#include <functional>
#include <map>

#include <GeoMath.h>
#include "juk_msg/juk_dji_gps_msg.h"
#include "juk_msg/juk_dji_device_status_msg.h"
#include "juk_msg/juk_control_dji_msg.h"
#include "juk_msg/juk_set_target_data_msg.h"
#include "juk_msg/juk_position_data_msg.h"
#include "juk_msg/juk_navigation_actions_msg.h"
#include <juk_msg/juk_aruco_module_action.h>
#include <juk_msg/juk_aruco_module_data.h>
#include <juk_msg/reach_msg.h>

#include "std_msgs/String.h"

#include "ArgParser.h"


#define c(color,str)  "\x1B["<<color<<"m" << str << "\033[0m" 



class NavigationNode
{
public:
	
	enum STATES ///< Основные состояния
	{
		IDLE = 0,
		///< аппарат удерживает ту позицию, в которой ему последний раз передали управление
	  FLY_SIMPLE = 1,
		///< аппарат летит в заданную точку по прямой
	FLY_SAFE = 2,
		///< аппарат летит в заданную точку на безопасной высоте (задается в параметрах запуска)
	LANDING_SIMPLE = 3,
		///< аппарат летит в указанную точку по прямой, а после совершает в ней вертикальную посадку
	  LANDING_ARUCO = 4,
		///< аппарат летит в указанную точку по прямой, а после совершает в ней посадку, ориентируясь по ArUco маркеру

	};
	enum SUB_STATES ///< Подсостояния
	{  
		NOTHING = 0, 
		///< подсостояние по умолчанию
		
	  FLY_SAFE_UP = 201,
		///< подсостояние FLY_SAFE - подъем на безопасную высоту
	  FLY_SAFE_CENTER = 202,
		///< подсостояние FLY_SAFE - полет к точке по прямой, если аппарат находится над точкой в коридоре радиусом 2м
		
	  LANDING_SIMPLE_FLY = 301,
		///< подсостояние LANDING_SIMPLE - полет к точке по прямой
	  LANDING_SIMPLE_LAND = 302,
		///< подсостояние LANDING_SIMPLE - посадка в указанной точке
			
	  LANDING_ARUCO_FLY = 401, 
		///< подсостояние LANDING_ARUCO - полет к точке по прямой
	   LANDING_ARUCO_LAND = 402,
		///< подсостояние LANDING_ARUCO - полет из указанной точки к каркеру 
	};
	
	enum ACTIONS
	{
		SET_HOMEPOINT = 1  ///< установка домашней точки
	};
	
	NavigationNode(int argc, char** argv);

private:
	
	ArgParser_Int params;   ///< парсер параметров запуска
	const int max_precision_uptime = 1000000000;   ///< максимальное время ожидания обновления прецизионных координат
	void gps_callback(const juk_msg::juk_dji_gps_msg::ConstPtr& input);   ///< обработчик события получения данных GPS A3
	void precision_gps_callback(const juk_msg::reach_msg::ConstPtr& in);   ///< обработчик события получения прецизионных данных GPS
	void set_target_callback(const juk_msg::juk_set_target_data_msg::ConstPtr& input);   ///< обработчик события установки новой целевой точки
	void action_process_callback(const juk_msg::juk_navigation_actions_msg::ConstPtr& input);   ///< обработчик события совершения действия
	void aruco_callback(const juk_msg::juk_aruco_module_data::ConstPtr& input);   ///< обработчик события получения данных о положении ArUco маркера

	//! Возвращает вектор управляющих параметров
	/*!
	    \param offset смещение относительно цели в метрах
	    \param current_velocity текущая скорость
	    \param course_need необходимый курс (градусы)
	    \param course_current текущий курс (градусы)
	    \param speed требуемая скорость
	    \param ctrl_mode режим торможения
	    \return сообщение, содержащее параметры управления А3
	*/
	
	juk_msg::juk_control_dji_msg calculateControl(GeoMath::v3 offset,
		GeoMath::v3 current_velocity,
		double course_need,
		double course_current,
		double speed,
		uint8_t ctrl_mode); 
	
	
	ros::Time node_start_time;   ///< время запуска ноды
	bool set_homepoint_flag = true;   ///< флаг, показывающий, нужно ли ставить хоумпоинт
	
	GeoMath::v3geo	homepoint;   ///< координаты хоумпоинта (А3)
	GeoMath::v3geo	homepoint_precision;   ///< координаты хоумпоинта (прецизионные)
	GeoMath::v3geo	current_point_abs;   ///< текущие координаты
	GeoMath::v3		current_velocity;   ///< текущая скорость
	GeoMath::v3     velocity_need; 
	GeoMath::v3     current_point_home;    ///< текущая точка относительно дома
	
	double homepoint_course;
	
	struct Target ///< Описывает целевую точку
	{
		GeoMath::v3geo	point_abs;    ///< абсолютные координаты цели
		uint8_t break_mode;   ///< режим торможения
		float cruising_speed;   ///< крейсерская скорость
		float accurancy;   ///< точность удержания точки
		float course;   ///< курс относительно севера
	}
	;
	
	struct ArUcoTarget ///< Описывает положение относительно маркера
	{
		GeoMath::v3 offset;   ///< смещение относительно маркера
		double course;   ///< курс
		ros::Time uptime;   ///<время последнего обновления данных
	};
	
	typedef struct CtrlStatus
	{
		juk_msg::juk_control_dji_msg msg;
		bool stable_now;
	};
	
Target target;   ///< Цель(координаты по А3)
	Target target_precision;    ///< Цель(координаты по Емлидке)
	Target current_target;   ///< Цель (принимает значения либо target, либо target_precision )
	Target sub_target;   ///< Подцель
	
	ArUcoTarget aruco_land;   ///< Маркер, на который нужно приземлиться
	
	int STATE;   ///< текущее состояние
	int SUB_STATE;   ///<  текущее подсостояние
	
	int flight_status;  ///< статус полета
		
	float yaw_rate;   ///< текущая угловая скорость
	
	bool stable_now;   ///< флаг, показывает, находится ли аппарат в целевой точке в данный момент
	bool stable_last;  ///< флаг, показывает, находился ли аппарат в целевой точке на предыдущий итерации цикла
	ros::Time stable_start;
	ros::Time home_uptime;
	double stable_time;
	
	ros::NodeHandle nh;
	ros::Publisher pub_dji_control;
	ros::Publisher pub_position_data;
	ros::Publisher pub_aruco_action;
	
	ros::Subscriber sub_dji_gps;
	ros::Subscriber sub_set_target;
	ros::Subscriber sub_precision_gps;
	ros::Subscriber sub_action_process;
	ros::Subscriber sub_aruco_data;
	
	uint8_t ctrl_mode;	 ///< текущий режим торможения
	
	//FIXME calvVel- ctrl_flag
	uint8_t ctrl_flag;  ///< режим управления А3 (устарело)
	
	
	juk_msg::juk_position_data_msg position_data; ///< полные данные о текущем положении
	juk_msg::juk_control_dji_msg output_dji; ///< параметры управления, передаваемые на А3
	
	GeoMath::v3geo precision_position; ///< текущие координаты (прецизионные)
	ros::Time precision_pos_uptime;  ///< время последнего обновления прецизионных координат
	
	int  precision_pos_quality; ///< уровень точности при определении прецизионных координат
	
	//FIXME calcVel - calcVel
	void calculateVelocity(double abs_speed, GeoMath::v3 offset, GeoMath::v3 current_velocity, uint8_t ctrl_mode);
	
	std::map<int, std::function<CtrlStatus()>> state_handlers;   ///< ключ - состояние, значение - функция-обработчик для данного состояния
	
	void init_handlers();
	
	void print_telemetry();
	const int telem_heigth = 10;
	ros::Time last_telemetry;
	
	std::map<int, std::string> state_map =  { {0,"IDLE"},{1, "FLY_SIMPLE"},{2, "FLY_SAFE"},{3, "LAND_SIMPLE"},{4, "LAND_ARUCO"}};
};

#endif // !__NAVIGATION_NODE__
