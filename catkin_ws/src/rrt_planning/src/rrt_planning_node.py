#!/usr/bin/env python

import rospy
import numpy as np
from math import atan2, cos, sin

from robotx_msgs.msg import ObstaclePoseList
from robotx_msgs.msg import Waypoint, WaypointList

class RRTPlanning(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
		
		self.pub_waypointList = rospy.Publisher("/waypointList", WaypointList, queue_size = 1)
		self.sub_obstacleList = rospy.Subscriber("/obstacleList", ObstaclePoseList, self.cb_obstacle, queue_size=1)
		self.sub_goal		  = rospy.Subscriber("/goal", WaypointList, self.cb_wplist, queue_size = 1)
		# Dimension
		self.X_DIM = None
		self.Y_DIM = None
		# Goal point
		self.q_goal = np.array([self.X_DIM, self.Y_DIM])
		# Inital point
		self.init = np.zeros(2)
		# Vehicle size
		self.car_length = 0.6
		# Walk distance
		self.delta_d = 0.8
		# Tolerance distance
		self.epsilon = 0.1
		# Standard derivation
		self.sigma = 10;
		# obstacles list
		self.obstacle_list = None
		# Waypoints size
		self.waypoint_size = 1
		# Waypoint list
		self.waypoint_list = WaypointList()
		self.waypoint_list.header.frame_id = "odom"
		# q_list size
		self.q_size = 1
		# q_list
		self.q_list = np.array([[0., 0., 1]])
		# q_near
		self.q_near = np.array([0., 0.])
		# q_rand
		self.published = False

	def cb_wplist(self, wp_list_msg):
		length = len(wp_list_msg.list)
		self.X_DIM = wp_list_msg.list[length-1].x
		self.Y_DIM = wp_list_msg.list[length-1].y

	def cb_obstacle(self, obstacle_msg):
		self.obstacle_list = obstacle_msg
		if self.published is False and self.X_DIM is not None:
			self._rrt_process()
		else:
			self.waypoint_list.header.stamp = rospy.Time.now()
			self.pub_waypointList.publish(self.waypoint_list)

	def _rrt_process(self):
		while (np.linalg.norm(self.q_goal - self.q_near) > self.epsilon):
			
			# Generate q_rand
			self.q_rand = np.array([np.random.normal(self.X_DIM, self.sigma),
					   	np.random.normal(self.Y_DIM, self.sigma)])
			self.q_near, index = self._near()
			if(self.q_near[0] > self.X_DIM):
				self.q_near[0] = self.X_DIM
			if(self.q_near[0] < 0):
				self.q_near[0] = 0
			if(self.q_near[1] > self.Y_DIM):
				self.q_near[1] = self.Y_DIM
			if(self.q_near[1] < 0):
				self.q_near[1] = 0
			if(self._is_hit_constrain(self.q_list[index], self.q_near) == 0):
				self.q_list = np.append(self.q_list, [[0, 0, 0]], axis = 0)
				self.q_list[self.q_size][0:2] = self.q_near
				self.q_list[self.q_size][2] = int(index+1)
				self.q_size = self.q_size + 1
		# end while 
		# Reverse tracking q_list
		waypoint_index_list = [self.q_size]
		child_index = int(self.q_size)
		parent_index = int(self.q_list[self.q_size-1][2])
		while (child_index != 1):
			self.waypoint_size = self.waypoint_size + 1
			waypoint_index_list.append(parent_index)
			temp = parent_index
			parent_index = int(self.q_list[temp-1][2])
			child_index = temp
		# end while
		self.waypoint_list.size = self.waypoint_size
		for i in range(self.waypoint_size, 0, -1):
			waypoint = Waypoint()
			waypoint.x = self.q_list[waypoint_index_list[i-1]-1][0]
			waypoint.y = self.q_list[waypoint_index_list[i-1]-1][1]
			print waypoint.x, waypoint.y
			self.waypoint_list.list.append(waypoint)
		self.waypoint_list.header.stamp = rospy.Time.now()
		self.pub_waypointList.publish(self.waypoint_list)
		self.published = True
		print "q_list", self.q_list
		print "waypoint index", waypoint_index_list
		print "waypoint list"
		for i in range(0, self.waypoint_size):
			print self.waypoint_list.list[i].x, self.waypoint_list.list[i].y
		rospy.loginfo("[%s] RRT path planning finished" %(self.node_name))

	# Given q_list and q_rand, return q_near and parent index
	def _near(self):
		min_dis = 1e6
		index = 0
		L, _ = self.q_list.shape
		
		for i in range(1, L):
			if np.linalg.norm(self.q_rand - self.q_list[i][0:2]) < min_dis:
				min_dis = np.linalg.norm(self.q_rand - self.q_list[i][0:2])
				index = i
			# end if
		# end for (i)
		temp = self.q_rand - self.q_list[index][0:2] 
		q_near = self.q_list[index][0:2] + temp / np.linalg.norm(temp) * self.delta_d

		return q_near, index
	# end _near

	# Check if the line segment from parent node to child node hit constrain, return 1 for true and 0 otherwise
	def _is_hit_constrain(self, q_now, q_near):
		result = 0
		counter = 1
		theta = atan2(q_near[1] - q_now[1], q_near[0] - q_now[0])
		_c = cos(theta)
		_s = sin(theta)
		t = np.zeros(16)
		s = np.zeros(16)
		while counter <= self.obstacle_list.size:
			for obs_index in range(1, self.obstacle_list.size):
				bound = [self.obstacle_list[obs_index].x - self.obstacle_list[obs_index].r / 2,
					 self.obstacle_list[obs_index].x + self.obstacle_list[obs_index].r / 2,
					 self.obstacle_list[obs_index].y - self.obstacle_list[obs_index].r / 2, 
					 self.obstacle_list[obs_index].y + self.obstacle_list[obs_index].r / 2]
  				
				t[0]  = (q_near[0]-bound[0]+self.car_length/2*_c)/_s
				s[0]  = (q_near[1]-bound[2]+t[0]*_c+self.car_length/2*_s)/(bound[3]-bound[2])
				t[1]  = (q_near[0]-bound[1]+self.car_length/2*_c)/_s
				s[1]  = (q_near[1]-bound[2]+t[1]*_c+self.car_length/2*_s)/(bound[3]-bound[2])
				t[2]  = (bound[2]-q_near[1]-self.car_length/2*_s)/_c
				s[2]  = (q_near[0]-bound[2]-t[2]*_s+self.car_length/2*_c)/(bound[1]-bound[0])
				t[3]  = (bound[3]-q_near[1]-self.car_length/2*_s)/_c
				s[3]  = (q_near[0]-bound[0]-t[3]*_s+self.car_length/2*_c)/(bound[1]-bound[0])
				
				t[4]  = (q_near[0]-bound[0]-self.car_length/2*_c)/_s
				s[4]  = (q_near[1]-bound[2]+t[4]*_c-self.car_length/2*_s)/(bound[3]-bound[2])
				t[5]  = (q_near[0]-bound[1]-self.car_length/2*_c)/_s
				s[5]  = (q_near[1]-bound[2]+t[5]*_c*self.car_length/2*_s)/(bound[3]-bound[2])
				t[6]  = (bound[2]-q_near[1]+self.car_length/2*_s)/_c
				s[6]  = (q_near[0]-bound[0]-t[6]*_s-self.car_length/2*_c)/(bound[1]-bound[0])
				t[7]  = (bound[3]-q_near[1]+self.car_length/2*_s)/_c
				s[7]  = (q_near[0]-bound[0]-t[7]*_s-self.car_length/2*_c)/(bound[1]-bound[0])

				t[8]  = (bound[0]-q_near[0]+self.car_length/2*_s)/_c
				s[8]  = (q_near[1]-bound[2]+t[8]*_s+self.car_length/2*_c)/(bound[3]-bound[2])
				t[9]  = (bound[1]-q_near[0]+self.car_length/2*_s)/_c	
				s[9]  = (q_near[1]-bound[2]+t[9]*_s-self.car_length/2*_c)/(bound[3]-bound[2])
				t[10] = (bound[2]-q_near[1]-self.car_length/2*_c)/_s
				s[10] = (q_near[0]-bound[0]+t[10]*_c-self.car_length/2*_s)/(bound[1]-bound[0])
				t[11] = (bound[3]-q_near[1]-self.car_length/2*_c)/_s
				s[11] = (q_near[0]-bound[0]+t[11]*_c-self.car_length/2*_s)/(bound[1]-bound[0])
				
				t[12] = (bound[0]-q_near[0]-self.car_length/2*_s)/_c
				s[12] = (q_near[1]-bound[2]+t[12]*_s-self.car_length/2*_c)/(bound[3]-bound[2])
				t[13] = (bound[1]-q_near[0]-self.car_length/2*_s)/_c
				s[13] = (q_near[1]-bound[2]+t[13]*_s-self.car_length/2*_c)/(bound[3]-bound[2])
				t[14] = (bound[2]-q_near[1]+self.car_length/2*_c)/_s
				s[14] = (q_near[0]-bound[0]+t[14]*_c-self.car_length/2*_s)/(bound[1]-bound[0])
				t[15] = (bound[3]-q_near[1]+self.car_length/2*_c)/_s
				s[15] = (q_near[0]-bound[0]+t[15]*_c+self.car_length/2*_s)/(bound[1]-bound[0])
				
				for parameter_checker in range(0,15):
					if(t[parameter_checker] >= -self.car_length/2 and 
					   t[parameter_checker] <= self.car_length/2 and
					   s[parameter_checker] >= 0 and
					   s[parameter_checker] <= 1):
						result = 1
					# end if
				# end for (parameter_checker)
			# end for (obs_index)
			counter = counter + 1
		#end while (counter)
		return result
	#end _is_hit_constrain
	
if __name__ == "__main__":
	rospy.init_node("rrt_planning_node", anonymous = False)
	rrt_planning = RRTPlanning()
	rospy.spin()
