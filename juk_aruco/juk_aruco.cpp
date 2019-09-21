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

static const std::string OPENCV_WINDOW = "Image window";






Vec3d rotationMatrixToEulerAngles(Mat &R)
{
     
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +  R.at<double>(1, 0) * R.at<double>(1, 0));
 
	bool singular = sy < 1e-6;  // If
 
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
	
	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;
	
	int mrk_id;
	int mrk_size;
	
public:
	ImageConverter()
		: it_(nh_)
	{
		mrk_id = 10;
		mrk_size = 180;
		
		action_sub = nh_.subscribe("JUK/ARUCO/ACTION",
			1,
			&ImageConverter::actionCb,
			this);

		image_pub_img = it_.advertise("JUK/ARUCO/IMG", 1);
		image_pub_gray = it_.advertise("JUK/ARUCO/GRAY", 1);
		image_pub_canny = it_.advertise("JUK/ARUCO/CANNY", 1);
		data_pub = nh_.advertise<juk_msg::juk_aruco_module_data>("JUK/ARUCO/DATA", 1);
		
		camera_matrix_ = (cv::Mat1f(3, 3) << 217.9576883804214, 0., 139.41597272515813,
									 0., 217.0956590832075, 123.53402954618754,
									 0., 0., 1.);
	
		dist_coeffs_ = (cv::Mat1f(8, 1) <<  -5.170122064089415,
										 -16.561517845457967,
										 -0.010622892474708852,
										 -0.007353905178370907,
										  88.52895898067152,
										 -5.337093535293237,
										 -14.999621815152844,
										 84.87525899538117);
		
		//image_sub_.shutdown();
		image_sub_ = it_.subscribe("/main_camera/image_raw/throttled",
			1,
			&ImageConverter::imageCb,
			this);
		cout << "SUB" << endl;
		
//				camera_matrix_ = (cv::Mat1f(3, 3) <<   318.2541080773673 ,
//													   0.0				 ,
//													   305.96305703665945,
//													   0.0				 ,
//													   318.6447775226269 ,
//													   219.22853938154097,
//													   0.0				 ,
//													   0.0				 ,
//													   1.0);
//	
//		dist_coeffs_ = (cv::Mat1f(8, 1) << -5.163903880082516,
//										   8.478007913776043,
//										   0.0001618818246359903,
//										   -0.001434442973040648,
//										   1.2663807298936525,
//										   -4.8256925147875425,
//										   6.686092517282609,
//										   4.397654490871483
//										  );

	}

	~ImageConverter()
	{
	}
	
	void actionCb(const juk_msg::juk_aruco_module_action::ConstPtr& msg)
	{
		
		mrk_id = msg->id;
		mrk_size = msg->size;
		cout << mrk_id << " " << mrk_size << " " << (int)msg->action << endl;
//		if ((int)msg->action == 1)
//		{
//			image_sub_.shutdown();
//			image_sub_ = it_.subscribe("/main_camera/image_raw/throttled",
//				1,
//				&ImageConverter::imageCb,
//				this);
//			cout << "SUB" << endl;
//		}
//		else
//		{
//			image_sub_.shutdown();
//			cout << "UNSUB" << endl;
//		}
		
		
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);		
		Mat img = cv_ptr->image.clone();

		int rotation_flag = 1;
		switch (rotation_flag)
		{
		case 0:
			break;
			
		case 1:
			rotate(img, img, ROTATE_90_CLOCKWISE);
			break;
		case 2:
			break;
		case 3:
			break;
		default:
			break;
		}
		
		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners, rejectedCandidates;
		Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);

		aruco::detectMarkers(img, markerDictionary, markerCorners, markerIds);
				
		
		for (int i = 0; i < markerCorners.size();i++)
		{
			if (markerIds[i] == mrk_id)
			{
				auto corn = markerCorners[i];
				vector<cv::Vec3d> rvecs, tvecs,euler;
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
				//cout << course * 57;
				
				for (auto val : tvecs)
				{
					//cout << val << " ";
				}
				//cout << tvecs[0][2]<<endl;
				juk_msg::juk_aruco_module_data data_msg;
				data_msg.x = tvecs[0][1];
				data_msg.y = tvecs[0][0];
				data_msg.z = tvecs[0][2];
				data_msg.course = course;
				
				data_pub.publish(data_msg);
				
			}
		}
		
		resize(img, img, Size(), 3, 3);		
		std_msgs::Header header; 
		header.stamp = ros::Time::now();
		
		
		
		cv_bridge::CvImage out_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC3, img);
		image_pub_img.publish(out_img.toImageMsg());
	}
};




int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "JUK_ARUCO");
	ImageConverter ic;
	
	
	ros::spin();
	return 0;
}


