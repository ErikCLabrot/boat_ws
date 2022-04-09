#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <vector>
cv_bridge::CvImagePtr imgPtr;
bool image_ready;
void imgCB(const sensor_msgs::Image::ConstPtr& img_in)
{
	imgPtr = cv_bridge::toCvCopy(img_in, sensor_msgs::image_encodings::RGB8);

	image_ready = true;
}

int main(int argc, char **argv)
{
	image_ready = false;
	
	ros::init(argc, argv, "image_rectification_node");
	ros::NodeHandle nh;
	ros::Rate checkRate = 100;
	
	std::string imageTopic = "ecam/image_raw";
	std::string rectTopic = "ecam/image_rect";
	ros::Subscriber imageSub = nh.subscribe(imageTopic, 1000, imgCB);

	ros::Publisher rectPub = nh.advertise<sensor_msgs::Image>(rectTopic, 1000);

	std::vector<double> distortionMat;
	std::vector<double> rawIntrins;
	nh.getParam("distortionMat", distortionMat);
	nh.getParam("rawIntrins", rawIntrins);
	std::cout << "1 ";
	cv_bridge::CvImage rect_image;
	cv::Mat rect;
	std::cout << "1 ";
	cv::Mat K(3,3, cv::DataType<double>::type);
	cv::Mat D(1,5, cv::DataType<double>::type);

	memcpy(K.data, rawIntrins.data(), rawIntrins.size()*sizeof(double));
	memcpy(D.data, distortionMat.data(), distortionMat.size()*sizeof(double));
	std::cout << "2 ";


	while(ros::ok())
	{
		if(image_ready)
		{
			std::cout << "Image ready\n";
			cv::Mat original = imgPtr->image;
			cv::undistort(imgPtr->image, rect_image.image, K, D);
			rect_image.header = imgPtr->header;
			rect_image.encoding = imgPtr->encoding;
			rectPub.publish(rect_image.toImageMsg());
			image_ready = false;
		}

		ros::spinOnce();
		checkRate.sleep();
	}
}