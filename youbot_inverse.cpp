//
// Simple demo program that calls the youBot ROS wrapper
//

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <iostream>
using namespace std;

ros::Publisher platformPublisher;
ros::Publisher armPublisher;
ros::Publisher gripperPublisher;

// create a brics actuator message with the given joint position values
brics_actuator::JointPositions createArmPositionCommand(std::vector<double>& newPositions) {
	int numberOfJoints = 5;
	brics_actuator::JointPositions msg;

	if (newPositions.size() < numberOfJoints)
		return msg; // return empty message if not enough values provided

	for (int i = 0; i < numberOfJoints; i++) {
		// Set all values for one joint, i.e. time, name, value and unit
		brics_actuator::JointValue joint;
		joint.timeStamp = ros::Time::now();
		joint.value = newPositions[i];
		joint.unit = boost::units::to_string(boost::units::si::radian);

		// create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		joint.joint_uri = jointName.str();

		// add joint to message
		msg.positions.push_back(joint);
	}

	return msg;
}

// create a brics actuator message for the gripper using the same position for both fingers
brics_actuator::JointPositions createGripperPositionCommand(double newPosition) {
	brics_actuator::JointPositions msg;

	brics_actuator::JointValue joint;
	joint.timeStamp = ros::Time::now();
	joint.unit = boost::units::to_string(boost::units::si::meter); // = "m"
	joint.value = newPosition;
	joint.joint_uri = "gripper_finger_joint_l";
	msg.positions.push_back(joint);		
	joint.joint_uri = "gripper_finger_joint_r";
	msg.positions.push_back(joint);		

	return msg;
}


// move platform a little bit back- and forward and to the left and right
void movePlatform() {
	geometry_msgs::Twist twist;

	// forward
	twist.linear.x = 0.05;  // with 0.05 m per sec
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// backward
	twist.linear.x = -0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// to the left
	twist.linear.x = 0;
	twist.linear.y = 0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// to the right
	twist.linear.y = -0.05;
	platformPublisher.publish(twist);
	ros::Duration(2).sleep();

	// stop
	twist.linear.y = 0;
	platformPublisher.publish(twist);
}

// move arm once up and down
void moveArm() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);


// move arm straight up. values were determined empirically
	jointvalues[0] = 2.95;
	jointvalues[1] = 2.59;
	jointvalues[2] = -2.44;
	jointvalues[3] = 1.73;
	jointvalues[4] = 2.88;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);

	ros::Duration(3).sleep();


	double px=0,pz=0,py=0;
	double theta_base=0, theta_2=0, theta_3=0, theta_4=0;
	double theta_link_1=0 , theta_link_2=0;
	double theta_link_1p=0, theta_link_2p=0;
	
	double x_cordinate=0,y_cordinate=0,z_cordinate =0;


//-------------------------------------------------------

	for (double t =0; t< 100; x++ ){
		
			double gam = M_PI*t/50;
			
			
			double px = 0.3;
			double py = 0.26+0.07*sin(gam);
			double pz = 0.07*cos(gam);
			  
			
			
			double theta_base = atan2 (py,px);
			
			
			
			
			if (py >=0){
			  theta_base = theta_base + 2.9624;
			  cout << " The value of  " << py <<"\n";
			  
			}
			else { 
			    theta_base = theta_base - 2.9624 + 5.8201;
			    cout << " Good \n ";  
			}
			
			
			if (theta_base > 5.8201){
			  cout << "NaN";
			}
			
			if (theta_base < 0){
			  cout << "NaN";
			}
			
			
			
			
			std::cout << theta_base << std::endl;

			double l1 = 0.302 - 0.147;
			double l2 = 0.437 - 0.302;
			double l3 = 0.655 - 0.437;
			
			
			double xc = sqrt(px*px +py*py);
			double zc = pz;
			double phi_c = 0;
			
			double d = sqrt (xc*xc + zc*zc);
			
			std::cout << " d = " << d << std::endl; 
			
			
			
			if (d > 0.508) {
			  std::cout << " Co-ordinate Points are out of the work space. \n" ;
			  std::cout << "Enter New Co-ordinates Points. \n" ;
			//break;
			}
			else if (d == 0.508){
			  double theta_link_1 = atan2(zc,xc) ;
			//break;
			}

			else {
						
			double xw = xc - l3*cos(phi_c);
			double zw = zc - l3*sin(phi_c);
			
			double alpha = atan2 (zw,xw);
			
			double cos_beta =  (l1*l1 + l2*l2 -xw*xw -zw*zw)/(2*l1*l2);
			double sin_beta = sqrt (abs(1 - (cos_beta*cos_beta)));
			double theta_link_2 = 3.1416 - atan2 (sin_beta , cos_beta) ;
			
			double cos_gama = (xw*xw + zw*zw + l1*l1 - l2*l2)/(2*l1*sqrt(xw*xw + zw*zw));
			double sin_gama = sqrt (abs (1 - (cos_gama * cos_gama)));
			
						
			double theta_link_1 = alpha - atan2(sin_gama, cos_gama);
			
			
			double theta_link_1p = theta_link_1 + 2*atan2 (sin_gama , cos_gama);
			double theta_link_2p = - theta_link_2;
			
			
			
			double theta_2 = theta_link_1p;
			double theta_3 = theta_link_2p;
			double theta_4 = (theta_2 + theta_3);

//apply the ik to reach destination positions
			jointvalues[0] = theta_base;
			jointvalues[1] = 2.59 - theta_2;
			jointvalues[2] = -2.43 - theta_3;
			jointvalues[3] = 1.7318 + theta_4;
			jointvalues[4] = 2.88;
			msg = createArmPositionCommand(jointvalues);
			armPublisher.publish(msg);

			# ros::Duration(2).sleep();	
			}
	}
	
		
}

void moveArmBack() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
	jointvalues[0] = 0.11;
	jointvalues[1] = 0.11;
	jointvalues[2] = -0.11;
	jointvalues[3] = 0.11;
	jointvalues[4] = 0.111;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);
	
	ros::Duration(2).sleep();




}


// open and close gripper
void moveGripper() {
	brics_actuator::JointPositions msg;
	
	// open gripper
	msg = createGripperPositionCommand(0.011);
	gripperPublisher.publish(msg);

	ros::Duration(3).sleep();

	// close gripper
	msg = createGripperPositionCommand(0);
	gripperPublisher.publish(msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "youbot_inverse");
	ros::NodeHandle n;

	platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
	gripperPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
	sleep(1);

	movePlatform();
	moveArm();
	moveArmBack();
	# moveGripper();

	sleep(1);
	ros::shutdown();

	return 0;
}
