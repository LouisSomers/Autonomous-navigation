# Autonomous navigation with dynamic obstacle avoidance
This is an implementation of autonomous navigation with dynamic obstacle avoidance in a simulation and the implementation of it in real-life. There is also an explanation of what is needed to be able to use all the sensors that were used.

On this page all the packages that are needed to run the simulation and to run it in real-life are found. The paper about this is found [here](https://github.com/LouisSomers/Autonomous-navigation/blob/main/MAP_Somers.Louis.pdf).

All the software is run using ROS Noetic and Gazebo 11. ROS Noetic can be installed from [here, ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). Gazebo 11 can be installed from [here, Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu).

## Simulation
A navigation method for obstacle avoidance in a static and dynamic environment is implemented in a simulation. All the files discussed below should be installed in a  catkin workspace ```catkin_ws ```, except for the ROS cartographer this should be installed in a seperate workspace that is isolated from the other packages, to prohibit interference between the packages.  Always make sure that the terminal your working in has sourced setup.bash. This can be done in the terminal by doing ``` source devel/setup.bash ```. This is done to set up the environment variables that are used by ROS and Gazebo.

### Simulator
The simulation is done in the USV simulater from Lenes and can be installed from his github: [USV simulator](https://github.com/jhlenes/usv_simulator). This simulator forms the basis of the work. It is already included in this github and thus does not need to be cloned from Lenes Github seperately.

### Complete coverage from Lenes
The starting point of the implementation of the navigation method was found in the work of Lenes for the mapping of the seabed in sheltered waters. His code can be found on his github via this link [complete coverage by Lenes](https://github.com/jhlenes/complete_coverage). Everything that is needed from this to let the simulation work is already included in this Github page.

### Cartographer packages
The cartographer package is no longer supported and needs to be installed from source. This can be done by following these instructions [cartographer_ros](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#)

Most probably, there will be an error that states that [libabsl-dev] is not available and it is not able to build cartographer. Then the dependance  on libabsl-dev need to be removed. ```<depend>libabsl-dev</depend>``` is found on line 46 in the package.xml file and needs to be removed.

When cartographer is build in an isolated workspace, one may not forget to source this workspace before running the code. Here, cartographer was installed isolated and thus the ```source install_isolated/setup.bash``` needed to be executed in the terminal before it is able to run the code.

### Detection and tracking of moving obstacles
The package to track moving obstacles can be found via this link [DATMO](https://github.com/kostaskonkk/datmo). Here a method to detect and track objects with 2D LIDAR measurements is implemented. This is used to the implement the prediction of the objects and to be able to do dynamic obstacle avoidance.

It can be ran seperatly from the rest with the following command ```roslaunch datmo datmo.launch```.

### Dynamic obstacles 
The dynamic obstacles that need to be avoided, can move throught the dynamic model plugin from [mrs_gazebo](https://github.com/ctu-mrs/mrs_gazebo_common_resources/tree/master/src/sensor_and_model_plugins?fbclid=IwAR12X8kWBT4RaV9GM4QtEslgTVqEDQwI0NRynIj4jqFpsvYlJRhzXDzDaJE). In this way the objects can move along predefined trajectories .
 These trajectories need to be written in a txt file as a list of "x y z roll pitch yaw velocity". The object will then move over these points with the defined roll, pitch and yaw and at the defined velocity. The dynamic ability can be added to each object in Gazebo with the following code.
 ```
 ...
    <plugin name="dynamic_target_plugin" filename="libMRSGazeboDynamicModelPlugin.so">
      <update_rate>30</update_rate>
      <initial_on>true</initial_on>
      <trajectory_file>path_to_trajectory.txt</trajectory_file>
      <loop_enabled>true</loop_enabled>
      <use_segmentation>true</use_segmentation>
      <use_directional_yaw>true</use_directional_yaw>
    </plugin>
  ...
 ```

## Real-life
For the real-life application. All the files are configured and ran on an Intel NUC. On this NUC, Ubuntu 20.04 and ROS Noetic are also installed. Furthermore, All the files from the simulation also need to be downloaded and build in the ```catkin_ws``` on this computer.

### RPLIDAR S1 
The RPLIDAR S1 is used in real-life for the detection of obstacles. The SDK to be able to run the RPLIDAR is found on [RPLIDAR_ros](https://github.com/Slamtec/rplidar_ros). By following the steps on that site, everything can be installed in your ``` catkin_ws```.

It is important when you want to use the LIDAR that it has the appropriate permissions. If the LIDAR is on port 
``` ttyUSB0 ``` the permissions can be granted by the following commands.

```
$ KERNEL=="ttyUSB*", MODE="0666"
$ sudo chmod 666/dev/ttyUSB0
ls -l /dev |grep ttyUSB #to check if it is on assigned port
```
The LIDAR can be tested to see if it works by running the following launch file ``` roslaunch rplidar_ros view_rplidar_s1.launch```. This launch file also opens a visual representation in RVIZ. To launch the node that sends the message without RVIZ, the following launch file needs to be ran ``` roslaunch rplidar_ros rplidar_s1.launch```

The launching of the node for the lidar is included in the overall launch file.

### RealSense D435i IMU
The sofware of the RealSense is installed from [RealSense SDK](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages). The necessary complementary files to enable all the capabilities of the D435i needs to be installed from [RealSense files](https://github.com/leggedrobotics/realsense-ros-rsl?fbclid=IwAR1UiCX0Tg-nl2NwrTvhwvcD0TtjAlMupl90yzODqZLOk2yu52TKlvp6EVw).

The D435i can be used as a camera to monitor the movements of the USV. This can be done in the RealSense-viewer application. This is launched by simply typing the command ```realsense-viewer``` in a terminal. The D435i will be automatically recognised.

To run the RealSense nodes, the following file needs to be launched. ```rs_camera.launch```. In this file, the IMU is disabled by default and needs to be enabled. To combine the information of the accelerometer and gyroscope a combination method needs to be chosen. Here, linear interpolation wa chosen. To check wether the information of the IMU is published this command can be used ```rostopic echo camera/imu```.


It is possible that the D435i is not found, when launching the viewer or a file. This can be caused by the KERNEL of Ubuntu that is used. The D435i does not work on the 5.15 0-70 and when restarting another KERNEL needs to be selected. When it was changed to 5.15.0-56 the D435i is found and everything works as it should.


### MAVROS
MAVROS is no longer supported for ROS noetic. The community has shifted to working with ROS2. However, MAVROS can still be installed from source. The instructions can be found on [MAVROS from source](https://github.com/mavlink/mavros/tree/master/mavros#installation). 

Mavros is needed to enable a communication link between the autopilot, in this case the Cube Orange and NUC. The Cube is configured with px4 software and thus the following launch file must be used ``` px4.launch``` In this file some parameters were adapted to enable the forwarding of GPS data towards the NUC and enable thruster commands from the NUC to the Cube.

### MAVLink (MAVProxy)
When one wants communication between the Cube and NUC without using ROS, MAVLink can be used. This establishes communication between both such that one can issue simple commands or read out data from the Cube on the computer. MAVLink is installed from [MAVLink_GitHub](https://github.com/mavlink/mavlink). Communication is established as follows
```
$ sudo chmod 666 /dev/ttyACMO
$ mavprox.py --master=/dev/ttyACMO
```
MAVLink is good when one wants to check the status of the USV or test some simple commands. MAVLink is all executed from the terminal. When a continuous stream of data of a sensor is needed in a script to position the USV, for example, it is better to use MAVROS.


### Trisonica wind sensor 
The software for the wind sensor can be found here [Trisonica wind sensor](https://github.com/vanbreugel-lab/trisonica_ros). The calibration for the wind sensor and the specifications can be found in the manual via this link [Trisonica manual](https://anemoment.com/wp-content/uploads/2019/04/Trisonica-Mini-User-Manual-Ap2019.pdf).

The wind sensor is ran with ``` rosrun trisonica_ros trisonica.py```. 

## Data analysis
There are also some other files on this Github page that serve to analyse the results obtained from the simulations and from hardware components. The results from these files are all explained in the thesis. This analysis includes matlab files about the accuracy of the detection and tracking of dynamic obstacles and the accuracy of the GPS used, as well as the calculation of the moment of inertia of the USV and the parameters of the used T200 thrusters.

## Running simulation
The simulation can be run with the following commands in this order and in seperate terminals:
```
  $ roslaunch sim_otter otter.launch
  $ roslaunch sim_guidance guidance.launch
  $ roslaunch usv_navigation usv_navigation.launch
```





