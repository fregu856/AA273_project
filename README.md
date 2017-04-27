# AA273_project

Below, the directory "AA273_project" is assumed to be placed in a directory called "AA273" in Home.  

Needed:  

Ubuntu 16.04.1  
ROS Kinetic  
ros-kinetic-turtlebot  
ros-kinetic-turtlebot-apps  
ros-kinetic-turtlebot-interactions  
ros-kinetic-turtlebot-simulator  
catkin

Catkin:  
Create a workspace:  
http://wiki.ros.org/catkin/Tutorials/create_a_workspace  
Add "source ~/AA273/AA273_project/catkin_ws/devel/setup.bash" to the bottom of ~/.bashrc ($ sudo nano ~/bashrc) for this line to be run everytime you open the terminal (otherwise you have to do it manually).  

Create a package in the workspace:  
$ cd AA273/AA273_project/catkin_ws/src  
$ catkin_create_pkg turtlebot_control std_msgs roscpp rospy  
$ cd AA273/AA273_project/catkin_ws  
$ catkin_make   

Create a scripts directory in the package:  
$ cd AA273/AA273_project/catkin_ws/src/turtlebot_control  
$ mkdir scripts  

Every script that you write must be made executable:  
$ chmod a+x script.py    
You should also build the package:  
$ cd AA273/AA273_project/catkin_ws  
$ catkin_make  

Simulation in Gazebo:  
$ cd AA273/AA273_project/catkin_ws/src  
$ git clone https://github.com/StanfordASL/asl_turtlebot.git  
$ cd AA273/AA273_project/catkin_ws    
$ catkin_make  
Add "export GAZEBO_MODEL_PATH=~/AA273/AA273_project/catkin_ws/src/asl_turtlebot/models" to the bottom of ~/.bashrc ($ sudo nano ~/bashrc) for this line to be run everytime you open the terminal (otherwise you have to do it manually).  

Test the simulation:  
$ roslaunch asl_turtlebot turtlebot_sim.launch  
If this doesn't work, open another terminal and do this FIRST:  
$ rosrun gazebo_ros gzclient  

Run a test controller to check that everything is working:  
Place "test_controller.py" in ~/AA273/AA273_project/catkin_ws/src/turtlebot_control/scripts, make it executable and build the package.  
$ roslaunch asl_turtlebot turtlebot_sim.launch  
$ rosrun turtlebot_control test_controller.py (in another terminal)  
The robot should now move to the coordinate (1, 1). 

*******

Run the AA274 project code:  
Create a directory called "launch" in catkin_ws/src/turtlebot_control  
Place "AA274_project.launch" in catkin_ws/src/turtlebot_control/launch  
$ roslaunch asl_turtlebot turtlebot_project_sim.launch  
$ roslaunch turtlebot_control AA274_project.launch  
$ rosrun turtlebot_control mission_publisher.py  

Tele-op:  
$ roslaunch kobuki_keyop keyop.launch  

*****

Launch the modified maze (maze3.world) and Gmapping:  
$ roslaunch asl_turtlebot turtlebot_maze.launch

**** 

To get the original verison of slam_karto running:  
- Clone slam_karto (https://github.com/ros-perception/slam_karto), open_karto (https://github.com/ros-perception/open_karto) and sparse_bundle_adjustment (https://github.com/ros-perception/sparse_bundle_adjustment) to catkin_ws/src.  
- For sparse_bundle_adjustment we also need to install SuiteSparse (https://github.com/jluttine/suitesparse), for which we in turn need to install OpenBLAS (https://github.com/xianyi/OpenBLAS).  

OpenBLAS:  
- Download the file from github, cd into the directory and run "$ sudo make" to build the package.
- Copy the files "libopenblas.so" and "libopenblas.so.0" from /opt/OpenBLAS/lib to usr/
lib (e.g. $ sudo cp libopenblas.so /usr/lib).  

SuiteSparse:  
- Download the file from github, cd into the directory and run "$ sudo make" to build the package.
- Copy all files from suitesparse-master/include to /usr/include and all files from suitesparse-master/lib to /usr/lib:  
$ cd Downloads/suitesparse-master  
$ sudo cp -a include/. /usr/include  
$ sudo cp -a lib/. /usr/lib  

spare_bundle_adjustment:  
- Change line 55 in catkin_ws/src/sparse_bundle_adjustment/include/sparse_bundle_adjustment/csparse.h from "#include "suitesparse/cs.h"" to "#include "cs.h"".  
- Change line 62 in catkin_ws/src/sparse_bundle_adjustment/include/sparse_bundle_adjustment/csparse.h from "#include "suitesparse/cholmod.h"" to "#include "cholmod.h"".  

Finally, build everything:  
$ cd catkin_ws  
$ catkin_make   

