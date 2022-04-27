#include <ros/ros.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include "boat_vision/calc_gps.h"

double heading;
sensor_msgs::NavSatFix gpsmsg;
bool gps_ready = false;
bool reading_ready = false;
void gpsCB(sensor_msgs::NavSatFix msg_in)
{
    gpsmsg.latitude = msg_in.latitude;
    gpsmsg.longitude = msg_in.longitude;
    gps_ready = true;
    std::cout << "Got fix\n";
}

void compassCB(std_msgs::Float64 reading)
{
    heading = reading.data;
    reading_ready = true;
    std::cout << "Got heading\n";
}


int main(int argc, char* argv[])
{
	bool loop = true;
	ros::init(argc, argv, "srv_test_node");
	ros::NodeHandle nh;
    ros::Rate checkRate = 100;

    std::string gps_topic = "/mavros/global_position/global";
    std::string compass_topic = "/mavros/global_position/compass_hdg";

    ros::Subscriber gps_sub = nh.subscribe(gps_topic, 1000, gpsCB);
    ros::Subscriber compass_sub = nh.subscribe(compass_topic, 1000, compassCB);
    ros::ServiceClient wpclnt = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    ros::ServiceClient distclnt = nh.serviceClient<boat_vision::calc_gps>("calc_gps");

    while(loop)
    {
    	std::cout << "Waiting for fix and heading\n";	    
    	if(gps_ready && reading_ready)
    	{ 
    		std::cout << "Fix and heading acquired\n";
    		loop = false;
    		boat_vision::calc_gps wpsrvc;
    	    wpsrvc.request.pos.latitude = gpsmsg.latitude;
            wpsrvc.request.pos.longitude = gpsmsg.longitude;


    		mavros_msgs::Waypoint point;
    		point.frame = 0;
            point.command = 16;
            point.param1 = 0;
            point.param2 = 1;
            point.param3 = 0;

    		mavros_msgs::WaypointPush mission;

    		//wp1
    		wpsrvc.request.dist = 1;
    		wpsrvc.request.head = heading;
    		if(distclnt.call(wpsrvc))
    		{
    			std::cout << "Calculating wp1\n";
    			point.x_lat = wpsrvc.response.coord.latitude;
    			point.y_long = wpsrvc.response.coord.longitude;
				std::cout << point.x_lat << " " << point.y_long << std::endl;
    		    mission.request.waypoints.push_back(point);
    		}
            else
                std::cout << "distance service call failed!\n";

    		//wp2
    		wpsrvc.request.dist = 1.41421356237;
    		if(heading < 45)
    			wpsrvc.request.head = 360-45;
    		else
    			wpsrvc.request.head = (heading - 45);
    		if(distclnt.call(wpsrvc))
    		{
    			std::cout << "Calculating wp2\n";
    			point.x_lat = wpsrvc.response.coord.latitude;
    			point.y_long = wpsrvc.response.coord.longitude;
    		   	std::cout << point.x_lat << " " << point.y_long << std::endl;
 				mission.request.waypoints.push_back(point);
    		}
            else
                std::cout << "distance service call failed!\n";

    		wpsrvc.request.dist = 1;
    		if(heading < 90)
    			wpsrvc.request.head = 360-90;
    		else
    			wpsrvc.request.head = (heading - 90);
    		if(distclnt.call(wpsrvc))
    		{
    			std::cout << "Calculating wp3\n";
    			point.x_lat = wpsrvc.response.coord.latitude;
    			point.y_long = wpsrvc.response.coord.longitude;
				std::cout << point.x_lat << " " << point.y_long << std::endl;    		  
    		    mission.request.waypoints.push_back(point);
    		}
            else
                std::cout << "distance service call failed!\n";

			std::cout << "Calculating wp4\n";
			point.x_lat = gpsmsg.latitude;
			point.y_long = gpsmsg.longitude;
			std::cout << point.x_lat << " " << point.y_long << std::endl;
		    mission.request.waypoints.push_back(point);
    		
		    if(wpclnt.call(mission))
		    	std::cout << "Mission pushed succesfully!\n";
            else
                std::cout << "Mission push failed!\n";
    	}
    	ros::spinOnce();
        checkRate.sleep();
    }

    return 0;
}
