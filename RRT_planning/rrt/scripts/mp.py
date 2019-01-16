#!/usr/bin/env python
#from __future__ import division
import numpy as np
import random
import sys

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import time 



def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.pub_msg = JointTrajectory()
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)


    '''This callback provides you with the current joint positions of the robot in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    def scale_distance(self, closest, random):
        K = 0.5
        unit_vector = np.true_divide(np.subtract(random, closest),np.linalg.norm(np.subtract(random, closest)))
        vector = np.add(unit_vector*K, closest)
        return vector


       

    '''This function is used to determine if the segment is valid.'''
    def is_segment_valid(self, q_closest, q_goal):
        step = 0.1
        vector = np.subtract(q_goal, q_closest)

        distance = np.linalg.norm(vector)
        
        #discret_num = max(np.ceil(abs(vector)/step))
        discrete_points = q_closest+np.outer(np.arange(step, distance,step), np.true_divide(vector,distance))


        for point in discrete_points:
            if self.is_state_valid(point) == False:
                return False
        return True



    def motion_planning(self, ee_goal):
        print "Starting motion planning"
        # get the translation of goal 
        R = tf.transformations.quaternion_matrix(np.array([ee_goal.rotation.x, ee_goal.rotation.y, ee_goal.rotation.z, ee_goal.rotation.w]))
        t = tf.transformations.translation_matrix(np.array([ee_goal.translation.x, ee_goal.translation.y, ee_goal.translation.z]))
        T = np.dot(t, R)

        # get the goal configuration space 
        q_goal = self.IK(T)
        if not q_goal:
            print "IK failed"
            return

        #start planning
        print "planning"
        
        # q_space is the Configuration space to hold all the valid states in it which starts from current joint angles. 2D np.array
        q_space = np.array([self.q_current])

        # nodes is used for storing the branches.
        nodes = np.array([])

        k = 0
        # in a specific time
        print 'start looping'
        start = rospy.get_rostime().secs

        while start<(start+200):

            # generate random numbers in the random space 2D np.array
            q_random = np.array([[random.uniform(-np.pi,np.pi) for i in range(self.num_joints)]])

            #2D np.array
            vector = q_space-q_random
            #find nearest point in the q_space 
            nearest_index = np.argmin(np.sqrt(np.sum((vector)**2, axis=1)))
            q_nearest = q_space[nearest_index]

            if self.is_segment_valid(q_nearest, q_random) == True:
                q_next = q_random[0]
                node  = RRTBranch(q_nearest,q_next)
                nodes = np.append(nodes, node)
                q_space = np.append(q_space, q_random, axis = 0)

                k=k+1
                print(k)
                if self.is_segment_valid(q_next, q_goal) == True:
                    #q_space = np.append(q_space, q_goal)
                    final_node = RRTBranch(q_random[0], q_goal)
                    nodes = np.append(nodes, final_node)
                    break

        print 'finishing looping'

        a = nodes[-1].parent
        route = [q_goal, a]
        while np.allclose(route[-1], self.q_current) == False:
            for i in nodes:
                if np.allclose(i.q, a) == True:
                    a = i.parent
                    route.append(a)

        # smooth the path

        start = route[0]
        smooth = [start]
        i = 2
        while i<len(route):
            if self.is_segment_valid(start, route[i]) == True:
                i = i+1
            else:
                smooth.append(route[i-1])
                start = route[i-1]
            
        smooth.append(self.q_current)
        smooth = smooth[::-1]

        print 'starting smoothing'

        # resample the path 2D
        sampled = np.array([self.q_current])
        for i in range(len(smooth)-1):
            #sampled = np.array([smooth[i]])
            sampled = np.append(sampled, self.traj_sample(smooth[i],smooth[i+1]),axis =0)


        print 'finishing sampling'
        self.pub_msg.joint_names = self.joint_names
        self.pub_msg.points = []
        for i in range(len(sampled)):
            p=JointTrajectoryPoint()
            p.positions = list(sampled[i])
            self.pub_msg.points.append(p)

        self.pub.publish(self.pub_msg)


        ######################################################

    def traj_sample(self, q_closest, q_goal):
        step = 0.3
        vector = np.subtract(q_goal, q_closest)
        distance = np.linalg.norm(vector)

        #discrete_points = np.array([q_closest])
        discrete_points =  np.outer(np.arange(step, distance, step), np.true_divide(vector,distance))+q_closest
        discrete_points = np.append(discrete_points, [q_goal],axis = 0)

        return discrete_points
        

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the joints of the robot arm, ordered from proximal to distal. If no IK solution is found, it returns an empty list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = np.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = np.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = np.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    '''This is a function which will collect information about the robot which has been loaded from the parameter server. It will populate the variables:
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state, or one that is free of collisions. The values in q[] are assumed to be values for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = np.zeros(self.num_joints)
        req.robot_state.joint_state.effort = np.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q) Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	    self.parent = parent
	    self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

