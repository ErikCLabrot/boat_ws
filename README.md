# boat_ws

boat_ws is the git repo for handling portability of the UWF Marine Robotics team's ROS code that is separate from the ros package index packages, and other miscellaneous packages that are a component of the project.

## Installation
Installing this package assumes you have completed the following
* Install ros
* Create a catkin_ws

Once the prerequisite workspace is configured and software is installed, this workspace can be cloned into the catkin_ws/src folder and made. The src folder of the workspace must be empty, including the CMakeLists.txt.
```bash
git clone https://github.com/ErikCLabrot/boat_ws .
```
Install these packages next:
* ros-melodic-velodyne - Install via apt using 
```bash
sudo apt-get install ros-melodic-velodyne
```
* mavros/mavlink - [Mavros Install](https://docs.px4.io/master/en/ros/mavros_installation.html)
* vectornav - [Vectornav Github](https://github.com/dawonn/vectornav)
* Hdl-Graph-Slam - Install via apt using 
```bash
https://github.com/koide3/hdl_graph_slam.git
```
* cam_lidar_calibration - [cam_lidar_calibration_Github](https://github.com/acfr/cam_lidar_calibration)

## License
None
