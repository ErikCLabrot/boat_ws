//TODO: Improve message instantiation. Generalize for multiple cameras.

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <vector>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ecam_info_node");

	std::string frame_id = "ecam_left";
	int seq = 0;
	int h;
	int w;
	std::string distortion;

	std::vector<double> distortionMat;
	std::vector<double> rawIntrins;
	std::vector<double> rectification;
	std::vector<double> projection;

	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	nh.getParam("camHeight", h);
	nh.getParam("camWidth", w);
	nh.getParam("distortion", distortion);
	nh.getParam("distortionMat", distortionMat);
	nh.getParam("rawIntrins", rawIntrins);
	nh.getParam("rectification", rectification);
	nh.getParam("projection", projection);

	ros::Publisher camInfo = nh.advertise<sensor_msgs::CameraInfo>(frame_id + "/ecam_info", 10);

	sensor_msgs::CameraInfo info_msg;

	info_msg.height = h;
	info_msg.width = w;
	info_msg.distortion_model = distortion;
	info_msg.D = distortionMat;

	//This is actually dumb. Rewrite this later or I'll eat your toes.
	for(int i = 0; i < rawIntrins.size(); i++)
		info_msg.K[i] = rawIntrins.at(i);
//	info_msg.K = rawIntrins;
	for(int i = 0; i < rectification.size(); i++)
		info_msg.R[i] = rectification.at(i);
//	info_msg.R = rectification;
	for(int i = 0; i < projection.size(); i++)
		info_msg.P[i] = projection.at(i);
//	info_msg.P = projection;

	info_msg.header.stamp = ros::Time::now();
	info_msg.header.seq = seq;
	info_msg.header.frame_id = "ecam_left";

	while(ros::ok())
	{
		camInfo.publish(info_msg);
		info_msg.header.seq = seq++;
		info_msg.header.stamp = ros::Time::now();
		loop_rate.sleep();
		ros::spinOnce();
	}	
	return 0;

}