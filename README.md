# youbot_ik
youbot inverse kinematics for the 5 Dof youbot arm.
The cpp code is modified based on the hello_world_demo of kuka youbot.

The diagrams are shown below to demonstrate how the coordinates are set up and how to obtain the geometric solution.

![coordinate][https://github.com/zyx124/youbot_ik/blob/master/diagram.jpg]



The arm of youBot only has 5 Dof and thus there are limitations in its motion in 3D space. 



The RRT_planning is a package to perform RRT in ROS. MoveIt service is used in the package. The KUKA_lwr, UR5 and youBot are created to perform the motion planning.

Instructions: 

​	roslaunch motion_planning kuka_lwr.launch

Then open a new terminal:

​	rosrun rrt mp.py

You can drag the marker using mouse and right click to choose to start planning. Several obstacles are also provided to test the algorithm.