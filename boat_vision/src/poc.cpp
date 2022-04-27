//inspire by https://github.com/acfr/cam_lidar_calibration/blob/master/src/assess_calibration.cpp

//C++ lib includes
#include <iostream>
#include <string>
#include <numeric>
#include <cv_basics/box.h>
#include <boat_vision/calc_gps.h>

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
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
//load in pointcloud
geometry_msgs::TransformStamped tf_msg;
pcl::PointCloud<pcl::PointXYZIR>::Ptr std_cloud(new pcl::PointCloud<pcl::PointXYZIR>);
pcl::PointCloud<pcl::PointXYZIR>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZIR>);
sensor_msgs::NavSatFix gpsmsg;
double heading;
cv_basics::box bounds;
bool bb_ready = false;
bool pcl_ready = false;
bool gps_ready = false;
bool reading_ready = true;


void gpsCB(sensor_msgs::NavSatFix msg_in)
{
    gpsmsg.latitude = msg_in.latitude;
    gpsmsg.longitude = msg_in.longitude;
    gps_ready = true;
}

void compassCB(std_msgs::Float64 reading)
{
    heading = reading.data;
    reading_ready = true;
}

void boxCB(const cv_basics::box::ConstPtr& bbIn)
{
    bounds.x = bbIn->x;
    bounds.y = bbIn->y;
    bounds.xw = bbIn->xw;
    bounds.yh = bbIn->yh;
    bb_ready = true;
}

void pclCB(const sensor_msgs::PointCloud2::ConstPtr& pcl_in)
{   
    pcl::PointCloud<pcl::PointXYZIR> temp_pcl;
    std::vector<int> indices;
    pcl::fromROSMsg(*pcl_in, temp_pcl);
    pcl::removeNaNFromPointCloud(temp_pcl, *std_cloud, indices);

    sensor_msgs::PointCloud2 pcl_buffer, tf_buffer;
    pcl::toROSMsg(*std_cloud, pcl_buffer);
    tf2::doTransform(pcl_buffer, tf_buffer, tf_msg);
    pcl::fromROSMsg(tf_buffer,*tf_cloud);
    pcl_ready = true;
}


int main(int argc, char* argv[])
{
    bool loop = true;
    double pi = 3.14159;

    //ROS setup
    ros::init(argc, argv, "point_projection_node");
    ros::NodeHandle nh;
    ros::Rate checkRate = 100;

    std::string pcl_topic;
    std::string gps_topic;
    std::string bb_topic;
    std::string compass_topic;

    mavros_msgs::WaypointPush mission;

    ros::Subscriber pcl_sub = nh.subscribe(pcl_topic, 1000, pclCB);
    ros::Subscriber gps_sub = nh.subscribe(gps_topic, 1000, gpsCB);
    ros::Subscriber compass_sub = nh.subscribe(compass_topic, 1000, compassCB);
    ros::Subscriber bb_sub = nh.subscribe(bb_topic, 1000, boxCB);

    ros::ServiceClient wpclnt = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    ros::ServiceClient distclnt = nh.serviceClient<boat_vision::calc_gps>("/calc_gps_server");

    //Lidar to Camera TF setup
    double K[9] = {720.5586936276926, 0.0, 638.2821742216654, 0.0, 722.4747616491345, 351.7413179250099, 0.0, 0.0, 1.0};
    cv::Mat cameramat;
    cameramat = cv::Mat::zeros(3,3, CV_64F);
    cameramat.at<double>(0, 0) = K[0];
    cameramat.at<double>(0, 2) = K[2];
    cameramat.at<double>(1, 1) = K[4];
    cameramat.at<double>(1, 2) = K[5];
    cameramat.at<double>(2, 2) = 1;

    tf2::Transform transform;
    tf2::Quaternion quat;
    tf2::Vector3 translation;

    quat.setRPY(-1.5344,0.034,-1.4888);
    translation.setX(0.0279);
    translation.setY(0.0750);
    translation.setZ(-0.1561);
    transform.setRotation(quat);
    transform.setOrigin(translation);

    tf_msg.transform.rotation.w = transform.inverse().getRotation().w();
    tf_msg.transform.rotation.x = transform.inverse().getRotation().x();
    tf_msg.transform.rotation.y = transform.inverse().getRotation().y();
    tf_msg.transform.rotation.z = transform.inverse().getRotation().z();
    tf_msg.transform.translation.x = transform.inverse().getOrigin().x();
    tf_msg.transform.translation.y = transform.inverse().getOrigin().y();
    tf_msg.transform.translation.z = transform.inverse().getOrigin().z();

    std::vector<int> indices;
    int index = 0;
    while(loop && ros::ok())
    {
        if(bb_ready && pcl_ready && gps_ready && reading_ready)
        {
            loop = !loop;
            //Compute lidar transform
            int midx = (bounds.x + bounds.xw)/2;
            int midy = (bounds.y + bounds.yh)/2;

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
                    indices.push_back(index);
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
            sensor_msgs::NavSatFix wp;
            boat_vision::calc_gps wpsrvc;

            wpsrvc.request.pos.latitude = gpsmsg.latitude;
            wpsrvc.request.pos.longitude = gpsmsg.longitude;

            mavros_msgs::Waypoint point;
            point.frame = 0;
            point.command = 16;
            point.param1 = 0;
            point.param2 = 1;
            point.param3 = 0;
            
            //Calculate angle 1.5m to 'left'
            double x2 = xcoord;
            double y2 = ycoord + 1.5;
            double dist2 = pow(x2*x2 + y2*y2, 0.5);
            double dot = (xcoord*x2) + (ycoord * y2);
            double det = (xcoord*y2) - (x2 * ycoord);
            double angle = atan2(det, dot);
            //calculate cur heading +/- angle
            angle = angle * (180/pi);
            if(angle > heading)
            {
                angle = 360 - angle;
            }
            else
                angle = heading - angle;
            //call coordinate service here
            wpsrvc.request.dist = dist2;
            wpsrvc.request.head = angle;
            if(distclnt.call(wpsrvc))
            {
                wp.latitude = wpsrvc.response.coord.latitude;
                wp.longitude = wpsrvc.response.coord.longitude;
            }


            point.x_lat = wp.latitude;
            point.y_long = wp.longitude;

            mission.request.waypoints.push_back(point);

            //calculate waypoint 1.5m back
            double x3 = xcoord + 1.5;
            double y3 = ycoord;
            double dist3 = pow(x3*x3 + y3*y3, 0.5);
            dot = (xcoord*x3) + (ycoord * y3);
            det = (xcoord*y3) - (x3 * ycoord);
            angle = atan2(det, dot);
            //calculate cur heading +/- angle
            angle = angle * (180/pi);
            if(angle > heading)
            {
                angle = 360 - angle;
            }
            else
                angle = heading - angle;
            //call coordinate service here
            wpsrvc.request.dist = dist3;
            wpsrvc.request.head = angle;
            if(distclnt.call(wpsrvc))
            {
                wp.latitude = wpsrvc.response.coord.latitude;
                wp.longitude = wpsrvc.response.coord.longitude;
            }

            point.x_lat = wp.latitude;
            point.y_long = wp.longitude;

            mission.request.waypoints.push_back(point);

            if(wpclnt.call(mission))
            {
                std::cout << "Mission sent successfully\n";
            }

        }
        ros::spinOnce();
        checkRate.sleep();
    }
    return 0;
}