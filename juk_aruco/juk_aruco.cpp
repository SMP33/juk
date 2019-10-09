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

Vec3d rotationMatrixToEulerAngles(Mat &R)
{
     
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +  R.at<double>(1, 0) * R.at<double>(1, 0));
 
	bool singular = sy < 1e-6;   // If
 
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
		cout << "SUB ARUCO" << endl;

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
		std_msgs::Header header; 

		aruco::detectMarkers(img, markerDictionary, markerCorners, markerIds);

	
		
		vector<GeoMath::v3> coordinates_all(0);
		vector<double> course_all(0);
		double angle = 0;
		
		for (int i = 0; i < markerCorners.size(); i++)
		{
			
			switch (markerIds[i])
			{
			case 18:
				mrk_size = 99;
				break;
			case 172:
				mrk_size = 540;
				break;
			default:
				continue;
				break;
			}
			auto corn = markerCorners[i];
			vector<cv::Vec3d> rvecs, tvecs, euler;
			vector<vector<Point2f>> corners;
			corners.push_back(corn);
			cv::aruco::estimatePoseSingleMarkers(corners,
				mrk_size/10,
				camera_matrix_,
				dist_coeffs_,
				rvecs,
				tvecs);
				
			cv::aruco::drawAxis(img, camera_matrix_, dist_coeffs_, rvecs, tvecs, mrk_size / 20);
				
			double course = rvec2Euler(rvecs[0])[2];
			
			Mat rvec(rvecs[0]), tvec(tvecs[0]);
			
			Mat R;
			cv::Rodrigues(rvec, R);
			R = R.t();
			
			Mat T = -R * tvec;

			
			Vec3d v3d(T);
			GeoMath::v3 offset(v3d[0], v3d[1], v3d[2]);
			
			course_all.push_back(course);
			coordinates_all.push_back(offset);
		}
		
		int marker_count = course_all.size();
		
		if (marker_count > 0 && marker_count == coordinates_all.size())
		{
			double course = 0;
			GeoMath::v3 offset;
			
			for (size_t i = 0; i < marker_count; i++)
			{
				course = course + course_all[i];
				offset = offset + coordinates_all[i];
			}
			
			course = course / (double)marker_count;
			offset = offset / (double)marker_count;
			
			offset = offset.rotateXY(-course);	
			
			GeoMath::v3 now_abs_mrk_pos(offset);
						
			if ((now_abs_mrk_pos - last_abs_mrk_pos).length_xyz()<150)
			{
				juk_msg::juk_aruco_module_data data_msg;
				
				data_msg.x = offset.y;
				data_msg.y = -offset.x;
				data_msg.z = offset.z;
				
				data_msg.course = course + angle*GeoMath::CONST.DEG2RAD;
				
				//cout << "ARUCO: \n\t" << "x: " << data_msg.x << " y: " << data_msg.y << " z: " << data_msg.z << " c: " << course*GeoMath::CONST.RAD2DEG <<  endl;
				data_pub.publish(data_msg);
			}
			

			
			last_abs_mrk_pos = now_abs_mrk_pos;
		}
		
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


