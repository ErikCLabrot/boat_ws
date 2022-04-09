import rospy
import cv2
import cv_bridge
import numpy
import math
import std_msgs
import tf
from sensor_msgs import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from image_geometry import PinholeCameraModel

cameraModel = PinholeCameraModel()

img_out = {}
T = []
cv_mat = []
cv_bridge_mat = {}
tf_received = False

def transformCB(mat_in):
