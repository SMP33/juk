#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <GeoMath.h>
#include <juk_msg/juk_aruco_module_action.h>
#include <juk_msg/juk_aruco_module_data.h>
 
using namespace cv;
using namespace std;

#ifndef CURSOR_H
#define	CURSOR_H

#define CLEAR(rows)               printf("\033[%02dA\033[0J",rows+1)
// ��������� �������
#define HOME                printf("\033[1;1H")
#define STORE_CURSOR        printf("\033[s")
#define RESET_CURSOR        printf("\033[u")
#define CLEAR_EOL           printf("\033[K")
#define CURSOR(row, column) printf("\033[%02d;%02dH", row, column)



// ����� �����
#define RESET_COLOR printf("\033[0m")

#define BLACK       0
#define RED         1
#define GREEN       2
#define YELLOW      3
#define BLUE        4
#define MAGENTA     5
#define CYAN        6
#define WHITE       7
#define UNDEF       -1

// ����� ����������
#define NONE        0
#define BOLD        1
#define DIM         2
#define UNDERLINE   4
#define BLINK       5
#define REVERSE     7

#define COLOR(bgcolor, fgcolor)         printf("\033[%02d;%02dm", (bgcolor + 30), (fgcolor + 40))
#define COLOR_A(bgcolor, fgcolor, attr) printf("\033[%02d;%02d;%1dm", (bgcolor + 30), (fgcolor + 40), attr)
#endif	/* CURSOR_H */

#define ROTATE_

void angle_normalize(double& rad)
{
	while (rad < 0)
	{
		rad += 2*GeoMath::CONST.Pi;
	}
	
	while (rad >= 2*GeoMath::CONST.Pi)
	{
		rad -= 2*GeoMath::CONST.Pi;
	}
}

struct mrk_params
{
	mrk_params()
	{
	}
	mrk_params(int size_, GeoMath::v3 abs_pos_) :
		size(size_),
		abs_pos(abs_pos_)
	{
	}
	
	int size=0;
	GeoMath::v3 pos_last;
	GeoMath::v3 pos_now;
	GeoMath::v3 abs_pos;
	
	double course=0;
	int qulity=0;
	int max_qulity=5;
};

Vec3d rotationMatrixToEulerAngles(Mat &R)
{
     
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +  R.at<double>(1, 0) * R.at<double>(1, 0));
 
	bool singular = sy < 1e-6;    // If
 
	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return Vec3d(x, y, z);    
}

Vec3d rvec2Euler(Vec3d rvec)
{
	Mat rot_mat;
	cv::Rodrigues(rvec, rot_mat);
				
	return rotationMatrixToEulerAngles(rot_mat);
}

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber action_sub;
	image_transport::Publisher image_pub_img;
	ros::Publisher data_pub;
	image_transport::Publisher image_pub_gray;
	image_transport::Publisher image_pub_canny;
	
	GeoMath::v3 last_abs_mrk_pos;
	
	
	ros::Time last_cb;
	
	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;
	
	int mrk_id;
	int mrk_size;
	
	map<int, mrk_params> markers;
	
public:
	ImageConverter()
		: it_(nh_)
	{
		last_cb = ros::Time::now();
		mrk_id = 172;
		mrk_size = 540;
		
		
		action_sub = nh_.subscribe("JUK/ARUCO/ACTION",
			1,
			&ImageConverter::actionCb,
			this);

		//		image_pub_img = it_.advertise("JUK/ARUCO/IMG", 1);
		//		image_pub_gray = it_.advertise("JUK/ARUCO/GRAY", 1);
				image_pub_canny = it_.advertise("JUK/ARUCO/CANNY", 1);
		data_pub = nh_.advertise<juk_msg::juk_aruco_module_data>("JUK/ARUCO/DATA", 1);
		image_pub_img = it_.advertise("JUK/ARUCO/IMG", 1);
		
		camera_matrix_ = (cv::Mat1f(3, 3) << 262.9282657089, 0.0000000000, 153.6920230483,
  0.0000000000, 262.4580575614, 121.4272230430,
  0.0000000000, 0.0000000000, 1.0000000000);
	
		dist_coeffs_ = (cv::Mat1f(5, 1) << 0.2008202636, -0.4864094868, 0.0008956347, -0.0014447575, 0.1407952918);
		
		image_sub_ = it_.subscribe("/main_camera/image_raw/throttled",
			1,
			&ImageConverter::imageCb,
			this);
		cout << "ARUCO DETECTION START" << endl;
//		cout << endl;
//		cout << endl;
		cout << endl;
		mrk_params mp;
		
		//mp.size = 540;
		markers[172] = mrk_params(540, GeoMath::v3(0, 0, 0));
		markers[18] = mrk_params(99, GeoMath::v3(0, 0, 0));
		markers[19] = mrk_params(99, GeoMath::v3(-10, 19, 0));
		markers[123] = mrk_params(99, GeoMath::v3(13.5, -13.5, 0));
		markers[97] = mrk_params(99, GeoMath::v3(18, 20, 0));
//		//mp.size = 99;
//		markers[0] = mrk_params(60, GeoMath::v3(-37+1000, 37, 0)/10); 
//		markers[1] = mrk_params(60, GeoMath::v3(37 + 1000, 37, 0) / 10); 
//		markers[2] = mrk_params(60, GeoMath::v3(-37 + 1000, -37, 0) / 10); 
//		markers[3] = mrk_params(60, GeoMath::v3(37 + 1000, -37, 0) / 10); 
		

	}

	~ImageConverter()
	{
	}
	
	void actionCb(const juk_msg::juk_aruco_module_action::ConstPtr& msg)
	{
		
		//		mrk_id = msg->id;
		//		mrk_size = msg->size;
		//		cout << mrk_id << " " << mrk_size << " " << (int)msg->action << endl;
		
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{		
		
		auto t1 = ros::Time::now();
		
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);		
		Mat img = cv_ptr->image.clone();

		int rotation_flag = 1;
		
		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners, rejectedCandidates;
		Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_1000);
		cv::Ptr<cv::aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
		parameters->adaptiveThreshWinSizeMin=10;
		std_msgs::Header header; 

		
		aruco::detectMarkers(img, markerDictionary, markerCorners, markerIds,parameters);

	
		
		vector<GeoMath::v3> position_vec(0);
		vector<GeoMath::v3> abs_pos_vec(0);
		vector<double> course_vec(0);
		
		
		
		
			
		for (int i = 0; i < markerCorners.size(); i++)
		{			
			if (markers.find(markerIds[i]) == markers.end())
			{
				vector<vector<Point2f>> corners_ = { markerCorners[i]};
				vector<int> ids_ = { markerIds[i]};
				aruco::drawDetectedMarkers(img, corners_, ids_);
				continue;
			}
			
			mrk_params& mp = markers[markerIds[i]];
			
			vector<Point2f> corn(markerCorners[i]);
			vector<cv::Vec3d> rvecs, tvecs, euler;
			vector<vector<Point2f>> corners(0);
			corners.push_back(corn);
			cv::aruco::estimatePoseSingleMarkers(corners,
				mp.size/10,
				camera_matrix_,
				dist_coeffs_,
				rvecs,
				tvecs);
				
			cv::aruco::drawAxis(img, camera_matrix_, dist_coeffs_, rvecs, tvecs, mp.size / 20);
			
				
			double course = rvec2Euler(rvecs[0])[2];
			
			Mat rvec(rvecs[0]), tvec(tvecs[0]);
			
			Mat R;
			cv::Rodrigues(rvec, R);
			R = R.t();
			
			Mat T = -R * tvec;
			
#ifdef ROTATE_
			Vec3d v3d(T);
			
			GeoMath::v3 offset(v3d[0], v3d[1], v3d[2]);
			offset.y = -offset.y;
			offset = offset.rotateXY(course);

			mp.pos_now = offset;
			angle_normalize(course);
			mp.course = course;
			
#else
			Vec3d v3d(tvec);
			GeoMath::v3 offset(v3d[0], v3d[1], v3d[2]);
#endif // ROTATE_
			Scalar color = Scalar(0, 0, 255);
						
			if ((mp.pos_now - mp.pos_last).length_xyz() < 200 )
			{
				mp.qulity++;
				
				if (mp.qulity >= mp.max_qulity)
				{
					color = Scalar(0, 255, 0);
					mp.qulity = mp.max_qulity;
					
					if (arcLength(markerCorners[i], true) < 150)
						{
							color = Scalar(0, 255, 255);
							Vec3d pos_simple(tvec);
							mp.pos_now = GeoMath::v3(-pos_simple[0], -pos_simple[1], pos_simple[2]);
						}
					
					position_vec.push_back(mp.pos_now + mp.abs_pos);
					abs_pos_vec.push_back(mp.abs_pos);
					course_vec.push_back(mp.course);
					
//					cout << endl << "ID: " << markerIds[i]<<" ";
//					cout << mp.pos_now << endl;
				}	
			}
			else
			{
				mp.qulity = 0;
			}
			
			mp.pos_last = mp.pos_now;
			
			vector<vector<Point>> contours(1);
			
			for (auto c : corners[0])
			{
				contours[0].push_back(c);
			}
			
			drawContours(img, contours, 0, color, 2);
		}
		
		int s_c = course_vec.size();
		int s_p = position_vec.size();
		
		if (s_c > 0 && s_c == s_p)
		{
			GeoMath::v2 direction(0, 0);
			double course = 0;
			GeoMath::v3 pos;
			
			for (int i = 0; i < s_c; i++)
			{
				direction = direction + GeoMath::v2(cos(course_vec[i]), sin(course_vec[i]));
				pos = pos + position_vec[i];
			}
			pos = pos / s_p;
			course = (direction).angle_xy(GeoMath::v2(1,0));
			
			juk_msg::juk_aruco_module_data data_msg;
			
			data_msg.x = -pos.y;
			data_msg.y = -pos.x;
			data_msg.z =  pos.z;
			data_msg.course = course;
			data_pub.publish(data_msg);
			
			//CLEAR(2);
			
			//cout  << "ArUco position:"<<endl << "\tx: " << data_msg.x << " y: " << data_msg.y << " z: " << data_msg.z << " c: " << course*GeoMath::CONST.RAD2DEG <<   endl << endl;
		}
		
		
		
//		
//		int marker_count = course_all.size();
//		
//		if (marker_count > 0 && marker_count == coordinates_all.size())
//		{
//			double course = course_all[0];
//			GeoMath::v3 offset;
//			
//			for (size_t i = 0; i < marker_count; i++)
//			{
//				offset = offset + coordinates_all[i];
//			}
//			offset = offset / (double)marker_count;
//#ifdef ROTATE_
//			offset.y = -offset.y;
//			
//			offset = offset.rotateXY(course);	
//#endif // ROTATE_
//			
//			GeoMath::v3 now_abs_mrk_pos(offset);
//						
//			if ((now_abs_mrk_pos - last_abs_mrk_pos).length_xyz() < 150)
//			{
//				juk_msg::juk_aruco_module_data data_msg;
//#ifdef ROTATE_
//				data_msg.x = -offset.y;
//				data_msg.y = -offset.x;
//				data_msg.z = offset.z;
//				
//				cout << "ARUCO: \n\t" << "x: " << data_msg.x << " y: " << data_msg.y << " z: " << data_msg.z << " c: " << course*GeoMath::CONST.RAD2DEG <<   endl;
//#else
//				data_msg.x = offset.y;
//				data_msg.y = offset.x;
//				data_msg.z = offset.z;
//#endif // ROTATE_
//				data_msg.course = course;
//				
//				data_pub.publish(data_msg);
//			}
//			
//
//			
//			last_abs_mrk_pos = now_abs_mrk_pos;
//		}
//		
		//resize(img, img, Size(), 3, 3);		
		
		header.stamp = ros::Time::now();
		
		line(img, Point(0, 0), Point(img.cols, img.rows), Scalar(255, 0, 255), 2);
		line(img, Point(0, img.rows), Point(img.cols, 0), Scalar(255, 0, 255), 2);
		
		//circle(img, Point2i(img.cols / 2, img.rows / 2), 20, Scalar(255, 0, 255), 2);
		
		
		//rotate(img, img, ROTATE_90_COUNTERCLOCKWISE);
		
		cv_bridge::CvImage out_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC3, img);
		image_pub_img.publish(out_img.toImageMsg());
		
		auto t2 = ros::Time::now();
		
		//cout <<"Delay: "<< t2 - t1 << endl;
	}
};




int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "JUK_ARUCO");
	ImageConverter ic;
	
	
	ros::spin();
	return 0;
}


