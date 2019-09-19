#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/aruco.hpp>

#include <GeoMath.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

bool in_area(double value, double center, double epsilon)
{
	return center*(1 - epsilon) <= value && center*(1 + epsilon) >= value;
}

double len(Point p1, Point p2)
{
	Point r = p2 - p1;
	return sqrt(r.x*r.x + r.y*r.y);
}

bool is_square(vector<Point> cnt, double acc)
{
	if (cnt.size() != 4)
		return false;
	
	double l[] = { len(cnt[0], cnt[1]), len(cnt[1], cnt[2]), len(cnt[2], cnt[3]), len(cnt[3], cnt[0]) };
	
	for (size_t i = 0; i < 4; i++)
	{
		if (l[i] < 10) return false;
		
		for (size_t j = 0; j < 4; j++)
		{
			double err = fabs(l[i] - l[j]);
			//cout <<"["<<i<<","<<j<<"] "<< err << " " << acc * l[i]<<endl;
			if(err > acc * l[i])
				return false;
			
		}
	}
	return true;
}

void drawBy4Corner(Mat &canvas, Mat src, vector<Point2f> markerCorners)
{
	Point2f c1[4];
	Point2f c2[4] =  {Point2f(0, 0), Point2f(0, 0), Point2f(0, 0), Point2f(0, 0) };
	
	for (int i = 0; i < markerCorners.size() && i<4; i++)
	{
		c2[i] = markerCorners[i];
	}
	
	
	
	Mat temp=src.clone();
	
	
	
	c1[0] = Point2f(0, 0);
	c1[1] = Point2f(src.cols, 0);
	c1[2] = Point2f(src.cols, src.rows);
	c1[3] = Point2f(0, src.rows);
	
	
	Mat M = getPerspectiveTransform(c1, c2);
	Mat mask_rgba;
	Mat mask_rgb;
		
	
		
	warpPerspective(temp, mask_rgba, M, canvas.size());		
		
	Mat mask(mask_rgba.size(), CV_8UC3, cv::Scalar(0, 0, 0)); 
	cvtColor(mask_rgba, mask_rgb, COLOR_BGRA2BGR);
		
	for (int i = 0; i < mask_rgba.rows; i++)
		for (int j = 0; j < mask_rgba.cols; j++)
		{
			if (mask_rgba.at<cv::Vec4b>(i, j)[3] == 255)
			{
				mask.at<cv::Vec3b>(i, j)[0] = 255;
				mask.at<cv::Vec3b>(i, j)[1] = 255;
				mask.at<cv::Vec3b>(i, j)[2] = 255;
			}
			else
			{
				mask.at<cv::Vec3b>(i, j)[0] = 0;
				mask.at<cv::Vec3b>(i, j)[1] = 0;
				mask.at<cv::Vec3b>(i, j)[2] = 0;
			}

		}
	
	mask_rgb.copyTo(canvas, mask);
	
}

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_img;
	image_transport::Publisher image_pub_gray;
	image_transport::Publisher image_pub_canny;
	
	Mat temp;

public:
	ImageConverter()
		: it_(nh_)
	{
		
		temp = imread("temp.png");
		cvtColor(temp, temp, COLOR_BGR2BGRA);
		
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/main_camera/image_raw/throttled",
			1,
			&ImageConverter::imageCb,
			this);
		image_pub_img = it_.advertise("/mrk/img", 1);
		image_pub_gray = it_.advertise("/mrk/gray", 1);
		image_pub_canny = it_.advertise("/mrk/canny", 1);

	}

	~ImageConverter()
	{
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);		
		Mat img = cv_ptr->image.clone();
		
		cv::Mat camera_matrix_;
		cv::Mat dist_coeffs_;
	
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
		
#ifdef CUSTOM
		Mat gray; 
		Mat canny;
		
		cvtColor(img, gray, COLOR_BGR2GRAY);
		GaussianBlur(gray, gray, Size(5, 5), 0);
		Canny(gray.clone(), canny, 40, 40);
		
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		
		findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		
		vector<vector<Point>> res_cnt(0);
		for (auto cnt : contours)
		{   
			vector<Point> cnt_approx;
			double epsilon = 0.02*arcLength(cnt, true);
			approxPolyDP(cnt, cnt_approx, epsilon, true);
			if (is_square(cnt_approx, 0.3))
				res_cnt.push_back(cnt_approx);
		}
			
		//Draw contours
		drawContours(img, contours, -1, Scalar(255, 0, 255));
		drawContours(img, res_cnt, -1, Scalar(255, 255, 0), 5);
		// Output modified video stream
		std_msgs::Header header; 
		header.stamp = ros::Time::now();
		
		cv_bridge::CvImage out_gray = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, gray);
		cv_bridge::CvImage out_canny = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, canny);
		
		cv_ptr->image = img;
		image_pub_img.publish(cv_ptr->toImageMsg());
		image_pub_canny.publish(out_canny.toImageMsg());
		image_pub_gray.publish(out_gray.toImageMsg()); 
#else
		//img = imread("m5.png");

		
		vector<int> markerIds;
		vector<vector<Point2f>> markerCorners, rejectedCandidates;
		Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);

		aruco::detectMarkers(img, markerDictionary, markerCorners, markerIds);
		
		
		//cvtColor(img, img, COLOR_BGR2BGRA);
		
		for (auto c : markerCorners)
		{
			drawBy4Corner(img, temp,c);
		}

		//cvtColor(img, img, COLOR_BGRA2BGR);
		
//		for (auto c : markerCorners)
//		{
//			vector<cv::Vec3d> rvecs, tvecs;
//			vector<vector<Point2f>> corners;
//			corners.push_back(c);
//			cv::aruco::estimatePoseSingleMarkers(corners,
//				0.15,
//				camera_matrix_,
//				dist_coeffs_,
//				rvecs,
//				tvecs);
//			drawBy4Corner(img, temp, c);
//			cv::aruco::drawAxis(img, camera_matrix_, dist_coeffs_, rvecs, tvecs, 0.15);
//		}
	

		
	
		
		//aruco::drawDetectedMarkers(img, markerCorners, markerIds);
		

		
		resize(img, img, Size(), 3, 3);		
		std_msgs::Header header; 
		header.stamp = ros::Time::now();
		
		cv_bridge::CvImage out_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC3, img);
		image_pub_img.publish(out_img.toImageMsg());
#endif // CUSTOM

	}
};



void createArucoMarkers()
{

	// Create image to hold the marker plus surrounding white space
	Mat outputImage(700, 700, CV_8UC1);
	// Fill the image with white
	outputImage = Scalar(255);
	// Define an ROI to write the marker into
	Rect markerRect(100, 100, 500, 500);
	Mat outputMarker(outputImage, markerRect);

	// Choose a predefined Dictionary of markers
	Ptr< aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);
	// Write each of the markers to a '.jpg' image file
	for(int i = 0 ; i < 99 ; i++)
	{
		cout << "Generage id: " << i << endl;
		//Draw the marker into the ROI
		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		ostringstream convert;
		string imageName = "4x4Marker_";
		convert << imageName << i << ".jpg";
		// Note we are writing outputImage, not outputMarker
		imwrite(convert.str(), outputImage);
	}
}

int main(int argc, char** argv)
{
	
	//createArucoMarkers();
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	
	
	ros::spin();
	return 0;
}


