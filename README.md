# Inverse Dynamics with Force Plate in Robotics
This repository contains the project files which arose within the framework of a bachelor thesis by one [student](https://github.com/deralbert) within the time frame of four months in the year 2023 in Karlsruhe, Germany. \
The bachelor thesis took place as part of a degree at the [KIT Department of Informatics](https://www.informatik.kit.edu/english/index.php). \
The experiments were carried out with the kind support of the employees at the [Robot and Human Motion Lab of the DHBW Karlsruhe, Germany (only available in german)](https://www.karlsruhe.dhbw.de/rahmlab/uebersicht.html) within their premises.


## Hardware and associated Software
- Vicon motion capture system
	- [Vicon Vero cameras](https://www.vicon.com/hardware/cameras/vero/)
	- [Vicon Lock Lab](https://www.vicon.com/hardware/devices/lock/) serving as input device from the AMTI amplifier
	- [Vicon Tracker](https://www.vicon.com/software/tracker/)  3.10
	- [Vicon Datastream SDK](https://www.vicon.com/software/datastream-sdk/) 1.12 (already [included in](ViconDataAcquisition/ViconLib1.12) the repository)
- [Universal Robots UR5e](https://www.universal-robots.com/products/ur5-robot/) robot
- AMTI hardware
	- [AMTI Optima BMS400600-2K](https://www.amti.biz/product/bms400600/) force plate
	- [Optima-SC (OPT-SC) Amplifier](https://www.amti.biz/product/optima-sc/)


## Dependencies
- [Kubuntu 20.04 LTS (Focal Fossa)](https://cdimage.ubuntu.com/kubuntu/releases/20.04/release/). Ubuntu version matters for ROS. See http://wiki.ros.org/noetic/Installation.


### Ubuntu packages
- [ros-noetic-desktop-full](http://wiki.ros.org/noetic/Installation/Ubuntu) 1.5.0
	- [ros-noetic-catkin](http://wiki.ros.org/catkin) 0.8.10
	- [ros-noetic-rosbag](http://wiki.ros.org/rosbag) 1.16.0
	- [ros-noetic-rospy](http://wiki.ros.org/rospy) 1.16.0
- [PlotJuggler](https://plotjuggler.io/) with ros integration [ros-noetic-plotjuggler 3.7.1](https://github.com/facontidavide/PlotJuggler/tree/3.7.1#debian-packages-for-ros-user)
- [Python](https://www.python.org/) 3.8.10
- [pip3](https://pip.pypa.io/en/stable/) 20.0.2
- [g++](https://gcc.gnu.org/) 9.4.0
- [cmake](https://cmake.org/) 3.26.4


### Python packages
Show local version of lib: `pip3 show <libname>` \
Install lib: `pip3 install <libname>` \
Update lib: `pip3 install -U <libname>`

- [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) 1.5.6
- [urdf2casadi](https://github.com/mahaarbo/urdf2casadi/tree/fc4232d7a095f078be0a3435cee3c1d4ef1cb8a0)
- [casadi](https://web.casadi.org/get/) 3.5.5
- [matplotlib](https://matplotlib.org/stable/users/installing/index.html) 3.7.2
- [pandas](https://pandas.pydata.org/docs/getting_started/install.html#installing-from-pypi) 2.0.3
- [numpy](https://numpy.org/install/) 1.24.3
- [scipy](https://scipy.org/install/) 1.10.1
- [varname](https://github.com/pwwang/python-varname) 0.11.2
- rosbag 1.16.0 (via [Ubuntu package](#ubuntu-packages))
- rospy 1.16.0 (via [Ubuntu package](#ubuntu-packages))
- ([sympy](https://www.sympy.org/en/index.html) 1.12)
- ([reloading](https://github.com/julvo/reloading) 1.1.2)


## Execution
#### Generate [plots](Plots):
- `cd` into [Scripts/Pipelines](Scripts/Pipelines).
- Execute the respective script via `python3 <scriptname>.py`.


#### Collect data with rosbag:
- `cd` into [catkin_ws](catkin_ws).
- Execute `catkin_make` once.
- Execute `source ./devel/setup.bash`. Rosnodes will need this command before their execution.
- Start `roscore` and all the necessary ros nodes. For some tasks, there are already bash scripts within the directory.
- Save data via `rosbag record`.
	- If you want to use the data afterwards, make sure to put the bagfiles in the [data directory](Data) intended for them.


#### Analyze time series visually with PlotJuggler:
- Start `roscore`.
- Start PlotJuggler via `rosrun plotjuggler plotjuggler`.
- You can use some PlotJuggler layouts saved in [catkin_ws](catkin_ws). Their filename extension is `.xml`.
- To use PlotJuggler live, start it together with the respective ros nodes when collecting data.
- To use PlotJuggler on saved data:
	- Execute `rosbag play -l <bagname>`.
	- Optionally, start some ros nodes that will work with the topics from the rosbags.

