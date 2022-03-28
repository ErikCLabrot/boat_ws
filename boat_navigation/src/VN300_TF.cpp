#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void tf_CB(const nav_msgs::Odometry::ConstPtr& VN300_Odom)
{
	tf::TransformBroadcaster tf_broadcaster;

	geometry_msgs::TransformStamped odom_tf;
	odom_tf.header.frame_id = "odom";//VN300_Odom->header.frame_id;
	odom_tf.child_frame_id = "base_link";//VN300_Odom->child_frame_id;

	odom_tf.transform.translation.x = VN300_Odom->pose.pose.position.x;
	odom_tf.transform.translation.y = VN300_Odom->pose.pose.position.y;
	odom_tf.transform.translation.z = VN300_Odom->pose.pose.position.z;

	odom_tf.transform.rotation = VN300_Odom->pose.pose.orientation;

	odom_tf.header.stamp = ros::Time::now();

	tf_broadcaster.sendTransform(odom_tf);

	std::cout << "Broadcast sent\n";

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "VN300_TF_Node");

	ros::NodeHandle nh;
	ros::Subscriber odom_in = nh.subscribe("vectornav/Odom", 1000, tf_CB);
	ros::spin();
}