# ESVO: Event-based Stereo Visual Odometry

**ESVO** is a novel pipeline for real-time visual odometry using a stereo event-based camera. Both the proposed mapping and tracking methods leverage a unified event representation (Time Surfaces), thus, it could be regarded as a ''direct'', geometric method using raw event as input.




# 1. Installation
* Ubuntu 20.04 LTS + ROS Noetic + OpenCV 4

## 1.1 Driver Installation

Reference at [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros). Do the steps below before moving on to the next step. 

	$ sudo apt-get install ros-noetic-camera-info-manager
 	$ sudo apt-get install ros-noetic-image-view
	$ sudo add-apt-repository ppa:inivation-ppa/inivation
	$ sudo apt-get update
	$ sudo apt-get install libcaer-dev
	$ sudo apt-get install python3-catkin-tools
 	$ mkdir -p catkin_ws/src
	$ cd catkin_ws
 	$ catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
  	$ cd src
	$ git clone https://github.com/catkin/catkin_simple.git
 	$ git clone https://github.com/uzh-rpg/rpg_dvs_ros.git
	$ cd ..
 	$ conda create --name AEvIA python=3.8
  	$ conda activate AEvIA
   	$ pip3 install pyyaml
	$ pip install rospkg
	$ pip install empy==3.3.4
	$ catkin build
  
## 1.2 Dependencies Installation

You should have created a catkin workspace in Section 1.1. If not, please go back and create one.

**Clone this repository** into the `src` folder of your catkin workspace.

	$ cd src 
	$ git clone https://github.com/HKUST-Aerial-Robotics/ESVO.git

Dependencies are specified in the file [dependencies.yaml](dependencies.yaml). They can be installed with the following commands from the `src` folder of your catkin workspace:

	$ sudo apt-get install python3-vcstool
	$ vcs-import < ESVO/dependencies.yaml

The previous command should clone the the repositories into folders called *catkin_simple*, *glog_catkin*, *gflags_catkin*, *minkindr*, etc. inside the `src` folder of your catking workspace, at the same level as this repository (ESVO).

You may need `autoreconf` to compile glog_catkin. To install `autoreconf`, run
    
	$ sudo apt-get install autoconf

Note that above command may change on different version of Ubuntu. 
Please refer to https://askubuntu.com/a/269423 for details.


**yaml-cpp** is only used for loading calibration parameters from yaml files:

	$ git clone https://github.com/jbeder/yaml-cpp.git
	$ cd yaml-cpp
	$ mkdir build && cd build && cmake -DYAML_BUILD_SHARED_LIBS=ON ..
	$ make -j18

Other ROS dependencies should have been installed in Section 1.1. 
If not by accident, install the missing ones accordingly.
Besides, you also need to have `OpenCV` (3.2 or later) and `Eigen 3` installed.

## 1.3 ESVO Installation
run

	$ catkin build esvo_time_surface esvo_core
	$ source ~/catkin_ws/devel/setup.bash

# 2. Usage
To run the pipeline, you need to download rosbag files from the [ESVO Project Page](https://sites.google.com/view/esvo-project-page/home).

## 2.1 esvo_time_surface
This package implements a node that constantly updates the stereo time maps (i.e., time surfaces). To launch it independently, open a terminal and run the command:

    $ roslaunch esvo_time_surface stereo_time_surface.launch
    
To play a bag file, go to `esvo_time_surface/launch/rosbag_launcher` and modify the path in 
`[bag_name].launch` according to where your rosbag file is downloaded. Then execute

    $ roslaunch esvo_time_surface [bag_name].launch
    
## 2.2 esvo_core 
This package implements the proposed mapping and tracking methods. The initialization is implemented inside the mapping part. To launch the system, run

    $ roslaunch esvo_core system_xxx.launch

This will launch two *esvo_time_surface nodes* (for left and right event cameras, respectively), the mapping node and the tracking node simultaneously. Then play the input (already downloaded) bag file by running

    $ roslaunch esvo_time_surface [bag_name].launch
    
To save trajectories at anytime, go to another terminal and terminate the system by

    $ rosparam set /ESVO_SYSTEM_STATUS "TERMINATE"
    
You need to set the path in `/cfg/tracking_xxx.yaml` to which the result file will be saved.

# 3. Parameters for Time Surface
## Time Surface
- `use_sim_time `: Set `True` for all offline experiments, which use 
simulation time. 
- `ignore_polarity `: Set `True` because polarity information is not 
used in the proposed methods. 
- `time_surface_mode `: Time surface rendering manner (0: Backward; 1: Forward). 0 indicates to
use the standard way to refresh the time surface. Please refer to the 
implementation for more details.
- `decay_ms `: The constant exponential decay parameter (unit: ms).
- `median_blur_kernel_size `: Determines the size of the kernel 
for denoising the time surface.
- `max_event_queue_len `: Determines the length of the event queue 
maintained at each coordinate.



# 4. Notes for Good Results
Real-time performance is witnessed on a Razor Blade 15 laptop (Intel® Core™ i7-8750H CPU @ 2.20GHz × 12).
* To get real-time performance, you need a powerful PC with modern CPUs which supports at least 6 threads. 
Remember to keep you computer cool!
* The mapping and tracking are loosely coupled, which indicates that the failure of anyone will lead to bad results of the other, and hence of the whole system.
* If the initialization does not look reasonably good, reset the system by clicking the checkbox `resetButton` in the dynamic reconfigure. This checker box is used as a button. Sorry for the bad GUI design.  
* If you use a PC with limited computational resources, you could slow down the playback of the rosbag by a factor, e.g.
    
    `$ rosbag play xxx.bag -r 0.5 --clock`
    
and modify the rate of the external clock (usd for synchronizing the stereo time surfaces) accordingly, e.g.
    
    `<node name="global_timer" pkg="rostopic" type="rostopic" args="pub -s -r 50 /sync std_msgs/Time 'now' ">`
    
In this example, the bag file is played at a factor of 0.5, and thus, the synchronization signal is set to 50 Hz accordingly. These modifications must be made accordingly such that the time surface is updated (refreshed) at 100 Hz in simulation time. You can check this by running,
   
   `$ rostopic hz /TS_left`
   `$ rostopic hz /TS_right`
   
They are both supposed to be approximately 100 Hz.
* The `esvo_core` is implemented using hyper-thread techniques. Please modify the number of threads used for mapping and tracking according to your PC's capability. The parameters can be found in `include/esvo_core/tools/utils.h`.
* The `esvo_time_surface` supports hyper-thread computation. We find this may be necessary when to deal with sensors with higher resolution than that of a DAVIS346 (346 x 260). The single thread implementation takes <= 5 ms on DAVIS240C (240 x 180) and <= 10 ms on DAVIS346 (346 x 260). 
We have evaluated the double-thread version on the mentioned PC platform, which gives <= 3 ms on 240 x 180 resolution and <= 6 ms on <= 346 x 260 resolution.
* Note that ESVO is non-deterministic, namely results may be different each time you run it on the same rosbag file. This is due to stochastic operations involved in the tracking, and also, the parallelism of the system. The performance differs according to the condition of your PC, e.g. you will get better efficiency if you turn off all other running programmes.


# 5. Datasets

The event data fed to ESVO needs to be recorded at remarkbly higher streaming rate than that in the default configuration (30 Hz) of the `rpg_dvs_ros` driver. This is due to the fact that the `esvo_time_surface` operates at 100 Hz. To refresh the time surfaces with the most current events to the utmost, a notably higher streaming rate is needed (e.g., 1000 Hz). The streaming rate can be either simply set in the hardware or modified via rewriting the bag. We provide a naive example in `/rosbag_editor` to show how. 

For convenience we provide a number of bag files, which have been rewritten to meet above requirement. They can be downloaded from the [ESVO Project Page](https://sites.google.com/view/esvo-project-page/home).



### References

* **[Event-based Stereo Visual Odometry](https://arxiv.org/abs/2007.15548)**, *Yi Zhou, Guillermo Gallego, Shaojie Shen*, IEEE Transactions on Robotics (T-RO) 2021.

* **[Semi-dense 3D Reconstruction with a Stereo Event Camera](https://arxiv.org/abs/1807.07429)**, *Yi Zhou, Guillermo Gallego, Henri Rebecq, Laurent Kneip, Hongdong Li, Davide Scaramuzza*, ECCV 2018.
