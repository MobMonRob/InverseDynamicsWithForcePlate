# Inverse Dynamics with Force Plate in Robotics
## Hardware
- Vicon motion capture system
	- [Vicon Vero cameras](https://www.vicon.com/hardware/cameras/vero/)
	- [Vicon Lock Lab](https://www.vicon.com/hardware/devices/lock/) as input device from the AMTI amplifier.
	- [Vicon Tracker Software](https://www.vicon.com/software/tracker/)
- [Universal Robots UR5e](https://www.universal-robots.com/products/ur5-robot/) robot
- AMTI hardware
	- [AMTI Optima BMS400600-2K](https://www.amti.biz/product/bms400600/#specifications) force plate
	- [Optima-SC (OPT-SC) Amplifier](https://www.amti.biz/product/optima-sc/)


## Dependencies
- Kubuntu 20.04
- g++ 9.4.0
- cmake 3.26.4
- [ros-noetic-desktop-full 1.5.0](http://wiki.ros.org/noetic/Installation/Ubuntu)
	- [ros-noetic-catkin 0.8.10](http://wiki.ros.org/catkin)
	- [ros-noetic-rosbag 1.16.0](http://wiki.ros.org/rosbag)
- [PlotJuggler](https://plotjuggler.io/) with ros integration [ros-noetic-plotjuggler 3.7.1](https://github.com/facontidavide/PlotJuggler/tree/3.7.1#debian-packages-for-ros-user)
- Python 3.8.10
- pip3 20.0.2


### Python packages
Show local version of lib: `pip3 show <libname>` \
Install lib: `pip3 install <libname>` \
Update lib: `pip3 install -U <libname>`

- ur_rtde 1.5.6
- matplotlib 3.7.2
- pandas 2.0.3
- numpy 1.24.3
- varname 0.11.2
- rosbag 1.16.0
- sympy 1.12
- casadi 3.5.5
- rospy 1.16.0
- [urdf2casadi](https://github.com/mahaarbo/urdf2casadi/tree/fc4232d7a095f078be0a3435cee3c1d4ef1cb8a0)
- reloading 1.1.2

