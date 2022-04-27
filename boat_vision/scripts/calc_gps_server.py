#!/usr/bin/env python
from sensor_msgs.msg import NavSatFix
from boat_vision.srv import calc_gps,calc_gpsResponse
import geopy
import geopy.distance
import math
import rospy

def service_handle(request):
	dist_feet = 3.281 * request.dist
	start_point = geopy.Point(request.pos.latitude, request.pos.longitude)
	end_point = geopy.distance.geodesic(feet=dist_feet).destination(start_point, request.head)
	msg = NavSatFix()
	msg.header.stamp = rospy.Time.now()
	msg.latitude = end_point.latitude
	msg.longitude = end_point.longitude
	return calc_gpsResponse(msg)

def service_server():
	rospy.init_node('calc_gps_server')
	s = rospy.Service('calc_gps', calc_gps, service_handle)
	rospy.spin()

if __name__ == "__main__":
	service_server()
