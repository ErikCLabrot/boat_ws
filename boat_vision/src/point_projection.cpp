#include <ros/ros.h>
//PCL inclusions
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
//ROS sensormsg types
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
//Matrix&Vector math
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud filteredPCL;
sensor_msgs::Image imageIn;
sensor_msgs::CameraInfo infoIn;

bool img_ready;
bool pcl_ready;
bool info_ready;

void imageCB(const sensor_msgs::Image::ConstPtr& img_in)
{
//	imageIn = &img_in;
	imageIn.height = img_in->height;
	imageIn.width = img_in->width;
	imageIn.encoding = img_in->encoding;
	imageIn.step = img_in->step;
	imageIn.data = img_in->data;
	img_ready = true;
}

void infoCB(const sensor_msgs::CameraInfo::ConstPtr& info_in)
{
	infoIn.height = info_in->height;
	infoIn.width = info_in->width;
//	infoIn.distortion_model = info_in->distortion_model;
//	infoIn.D = info_in->D;
//	infoIn.K = info_in->K;
//	infoIn.R = info_in->R;
	for(int i = 0; i < 12; i++)
		infoIn.P[i] = info_in->P[i];

	info_ready = true;
}

void pclCB(const sensor_msgs::PointCloud2::ConstPtr& pcl_in)
{	
	PointCloud temp_pcl;
	std::vector<int> indices;
	pcl::fromROSMsg(*pcl_in, temp_pcl);
	pcl::removeNaNFromPointCloud(temp_pcl, filteredPCL, indices);
	pcl_ready = true;
}

void pubProjection()
{

}

int main(int argc, char **argv)
{
	img_ready = false;
	pcl_ready = false;
	info_ready = false;

	std::vector<float> T;
	//Topic names, to be parameterized
	std::string imageTopic = "/ecam_left/image";
	std::string infoTopic = "ecam_left/cameraInfo";
	std::string pclTopic = "/velodyne_points";
	std::string projectionTopic = "/projected_points";
	//Ros initialization
	ros::init(argc, argv, "point_projection_node");
	ros::Rate checkRate = 100;
	ros::NodeHandle nh;

	//Get params
	nh.getParam("T", T);

	cv::Mat RT(4,4,CV_32FC4, T.data(), T.size());
	cv::Mat x(4,1, cv::DataType<double>::type);
	cv::Mat y(3,1, cv::DataType<double>::type);
	cv::Mat P(3,4, cv::DataType<double>::type);
	//Register subs
	ros::Subscriber imageSub = nh.subscribe(imageTopic, 1000, imageCB);
	ros::Subscriber infoSub = nh.subscribe(infoTopic, 1000, infoCB);
	ros::Subscriber pclSub = nh.subscribe(pclTopic, 1000, pclCB);

	//Register image pub
	ros::Publisher projectionPub = nh.advertise<sensor_msgs::Image>(projectionTopic, 1000);

	if(T.size() < 16)
	{
		std::cout << "Error! Rigid body transform matrix is not 4x4.\n";
		exit(1);
	}


	PointCloud transformed_cloud;
	std::vector<cv::Point> transformed_points;
	int loopCount = 0;
	while(ros::ok())
	{

		if(pcl_ready && img_ready && info_ready)
		{
			//Do transform

			//get Prect from caminfo
			P = cv::Mat(3,4, cv::DataType<double>::type, &infoIn.P);

			//Transform all points from lidar to cam frame
			//Could also try pcl::transformPointCloud
			for(auto iterator = filteredPCL.begin(); iterator != filteredPCL.end(); iterator++)
			{
				x.at<double>(0,0) = iterator->x;
				x.at<double>(1,0) = iterator->y;
				x.at<double>(2,0) = iterator->z;
				x.at<double>(3,0) = 1;

				//Formula from https://journals.sagepub.com/doi/full/10.1177/0278364913491297
				//R_Rect doesn't apply to monocular cams
				y = P * RT * x;

				cv::Point tf_pt;
				tf_pt.x = y.at<double>(0,0)/y.at<double>(0,2);
				tf_pt.y = y.at<double>(1,0)/y.at<double>(0,2);
				transformed_points.push_back(tf_pt);
			}

			//Construct openCV image to check point transformation. If it works, begin working on getting it backwards.

						
		}



		ros::spinOnce();
		checkRate.sleep();
		loopCount++;
	}




}