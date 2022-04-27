//inspire by https://github.com/acfr/cam_lidar_calibration/blob/master/src/assess_calibration.cpp

//C++ lib includes
#include <iostream>
#include <string>
#include <numeric>

//PCL Inclusions
#include "boat_vision/point_xyzir.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

//OpenCV Inclusions
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

//ros inclusions
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>


 
int main(int argc, char* argv[])
{
	double K[9] = {720.5586936276926, 0.0, 638.2821742216654, 0.0, 722.4747616491345, 351.7413179250099, 0.0, 0.0, 1.0};
    cv::Mat cameramat;
	int height = 720;
	int width = 1280;
	std::string imgpth = "/home/eriklabrot/boat_ws/src/boat_ws/boat_vision/data/pose16.png";
	std::string pclpth = "/home/eriklabrot/boat_ws/src/boat_ws/boat_vision/data/pose16_full.pcd";

	tf2::Transform transform;
	tf2::Quaternion quat;
	tf2::Vector3 translation;

	quat.setRPY(-1.5344,0.034,-1.4888);
	translation.setX(0.0279);
	translation.setY(0.0750);
	translation.setZ(-0.1561);
	transform.setRotation(quat);
	transform.setOrigin(translation);

    cameramat = cv::Mat::zeros(3,3, CV_64F);
    cameramat.at<double>(0, 0) = K[0];
    cameramat.at<double>(0, 2) = K[2];
    cameramat.at<double>(1, 1) = K[4];
    cameramat.at<double>(1, 2) = K[5];
    cameramat.at<double>(2, 2) = 1;

	//load in image
    std::cout << "Loading image\n";
	cv::Mat image = cv::imread(imgpth, cv::IMREAD_COLOR);
	//load in pointcloud
    pcl::PointCloud<pcl::PointXYZIR>::Ptr std_cloud(new pcl::PointCloud<pcl::PointXYZIR>);
    pcl::PointCloud<pcl::PointXYZIR>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZIR>);

    std::cout << "Loading pcl\n";
    pcl::io::loadPCDFile<pcl::PointXYZIR>(pclpth, *std_cloud);
    //Calculate transformed point cloud (source is inspiration)
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.transform.rotation.w = transform.inverse().getRotation().w();
    tf_msg.transform.rotation.x = transform.inverse().getRotation().x();
    tf_msg.transform.rotation.y = transform.inverse().getRotation().y();
    tf_msg.transform.rotation.z = transform.inverse().getRotation().z();
    tf_msg.transform.translation.x = transform.inverse().getOrigin().x();
    tf_msg.transform.translation.y = transform.inverse().getOrigin().y();
    tf_msg.transform.translation.z = transform.inverse().getOrigin().z();

    sensor_msgs::PointCloud2 pcl_buffer, tf_buffer;
    pcl::toROSMsg(*std_cloud, pcl_buffer);
    tf2::doTransform(pcl_buffer, tf_buffer, tf_msg);
    pcl::fromROSMsg(tf_buffer,*tf_cloud);
	//get pointcloud points in ROI
    int x = 590;
    int y = 170;
    int xw = 850;
    int yh = 460;
    int midx = (x + xw)/2;
    int midy = (y+yh)/2;
    int index = 0;
    //std::vector<pcl::PointCloud<pcl::PointXYZIR>::iterator> roi_maps;
    std::vector<int> indices;
    //Sourced directly from github
    std::cout << "For loop top\n";
    pcl::PointCloud<pcl::PointXYZIR>::const_iterator it;
    for(it = tf_cloud->begin(); it != tf_cloud->end(); it++)
    {
	    double tmpxC = it->x / it->z;
        double tmpyC = it->y / it->z;
        double tmpzC = it->z; 
        double dis = pow(it->x * it->x + it->y * it->y + it->z * it->z, 0.5);
        cv::Point2d planepointsC;
        int range = std::min(round((dis / 30.0) * 49), 49.0);


        // Applying the distortion
        double r2 = tmpxC * tmpxC + tmpyC * tmpyC;
        double r1 = pow(r2, 0.5);
        double a0 = std::atan(r1); //zero out distortion matrix for whatever reason. Not perfect but w/e
        planepointsC.x = (a0 / r1) * tmpxC;
        planepointsC.y = (a0 / r1) * tmpyC;

        planepointsC.x = cameramat.at<double>(0, 0) * planepointsC.x + cameramat.at<double>(0, 2);
        planepointsC.y = cameramat.at<double>(1, 1) * planepointsC.y + cameramat.at<double>(1, 2);

        if (planepointsC.y >= midy-10 and planepointsC.y < midy+10 and planepointsC.x >= midx-10 and planepointsC.x < midx and tmpzC >= 0 and std::abs(tmpxC) <= 1.35) 
        {
            int point_size = 2;
            cv::circle(image,
                cv::Point(planepointsC.x, planepointsC.y), point_size,
                CV_RGB(0,255,0));
            indices.push_back(index);
            //roi_maps.push_back(it - tf_cloud->begin());
        }
        index++;
    }

    //Take average of centerpoint coordinates
    double xcoord;
    double ycoord;
    double zcoord;
    if(indices.size() > 0)
    {
        for(int i = 0; i < indices.size(); i++)
        {
            it = std_cloud->begin() + indices.at(i);
            xcoord += it->x;
            ycoord += it->y;
            zcoord += it->z;
        }

        xcoord = xcoord/indices.size();
        ycoord = ycoord/indices.size();
        zcoord = zcoord/indices.size();
    }
    std::cout << xcoord << " " << ycoord << " " << zcoord << std::endl;
    cv::Mat resized_img;
    cv::resize(image, resized_img, cv::Size(), 0.75, 0.75);
    cv::imshow("ROI Points", resized_img);
    cv::waitKey(0);
	//print distance
	return 0;
}
