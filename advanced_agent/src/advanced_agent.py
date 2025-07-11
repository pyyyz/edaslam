#!/usr/bin/env python3

import sys
import math 
import rospy
import actionlib
import functions

from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult, MoveBaseAction
from actionlib_msgs.msg import GoalID, GoalStatusArray
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan 

class Explorer:

	def __init__(self):
		
		self.map = []
		self.laser = []
		self.pose_array = []
		self.topologic_map = []
		self.odom_pose = [0, 0, 0]
		self.topologic_entropy = []

		self.entropy = 4.567
		self.range_min = 0
		self.range_max = 0
		
		self.C = 2.5
		self.C1 = 6
		self.C2 = 20
		self.UMBRAL_ENTROPY = self.entropy
		
		self.save_goal = None
		self.has_result = False
		self.is_reclosing = False

		#Move Base Frontier Server
		self.server = actionlib.SimpleActionServer('move_base_frontier', MoveBaseAction, self.execute, False)
		self.server.start()

		#Move Base Client
		self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.client.wait_for_server()

		#Proxy Move Base Server
		self.get_plan = rospy.ServiceProxy('/move_base_planner/make_plan', GetPlan)

		#Publishers
		self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=20)
		self.pub_closing = rospy.Publisher('/custom/closing', Bool, queue_size=20)
		self.pub_reclosing = rospy.Publisher('/custom/reclosing', Bool, queue_size=20)
		self.pub_pose_array = rospy.Publisher('/custom/pose_array', PoseArray, queue_size=20)
		self.pub_status = rospy.Publisher('/move_base_frontier/status', GoalStatusArray, queue_size=20)
		self.pub_result = rospy.Publisher('/move_base_frontier/result', MoveBaseActionResult, queue_size=20)
		self.pub_feedback = rospy.Publisher('/move_base_frontier/feedback', MoveBaseActionFeedback, queue_size=20)


	def execute(self, goal):
		if not self.is_reclosing:
			total = 0
			i_s = []
			i = 0
			self.save_goal = goal
			
			actual_pose = self.odom_pose
			reverse = self.topologic_map[::-1]
			last = [actual_pose[0], actual_pose[1], 0]

			min_path = []
			for pose in reverse:
				real_distance = functions.get_distance(actual_pose, pose)
				topologic_distance = functions.get_distance(last, pose)
				total += topologic_distance

				if pose not in min_path:
					if real_distance < self.C1 and total > self.C2:
						i_s.append(pose)
					min_path.append(pose)

				last = pose
				i+=1

			cicle_close = len(i_s) > 0 and self.entropy > self.UMBRAL_ENTROPY

			bool_closing = Bool()
			
			if not cicle_close:
				bool_closing.data = False
				self.pub_closing.publish(bool_closing)
				rospy.loginfo("Robot action: Exploring")
				
				self.server.set_succeeded()
				self.client.send_goal(goal)
			else:
				bool_closing.data = True
				self.pub_closing.publish(bool_closing)
				rospy.loginfo("Robot action: Loop-Closing")

				new_pose = None
				new_distance = 9999

				for pose in i_s:
					distance = self.get_path_distance(actual_pose, pose)
					if distance < new_distance:
						new_pose = [pose[0], pose[1], 0]
						new_distance = distance


				if new_pose is not None:
					goal.target_pose.pose.position.x = new_pose[0]
					goal.target_pose.pose.position.y = new_pose[1]
					goal.target_pose.pose.position.z = 0

					self.topologic_map.append(new_pose)
					data_entropy = {
						"pose": new_pose,
						"entropy": self.entropy
					}
					self.topologic_entropy.append(data_entropy)

				self.server.set_succeeded()
				self.client.send_goal(goal)

				goal_pose = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, 0]

				h_te = self.UMBRAL_ENTROPY
				for item in self.topologic_entropy[::-1]:
					if item['pose'] == goal_pose:
						h_te = item['entropy']
						break

				while self.entropy > h_te and self.entropy > self.UMBRAL_ENTROPY and functions.get_distance(actual_pose, goal_pose) > 0.3:
					if actual_pose[0] != self.odom_pose[0] and actual_pose[1] != self.odom_pose[1]:
						actual_pose = self.odom_pose

	
	def get_is_visible(self, orientation, actual_pose, pose):
		orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		rotation = math.degrees(yaw)

		angle = functions.get_angle(actual_pose, pose)
		angle = int(angle - rotation) % 360
		real_distance = functions.get_distance(actual_pose, pose)
		is_visible = False

		if len(self.laser) > 0:
			laser_distance = self.laser[angle]
			is_visible = (str(laser_distance) != 'inf' and laser_distance > real_distance) or (str(laser_distance) == 'inf' and real_distance <= self.range_max)

		return is_visible


	def get_path_distance(self, goal1, goal2):
		start = PoseStamped()
		start.header.seq = 0
		start.header.frame_id = "map"
		start.header.stamp = rospy.Time.now()
		start.pose.position.x = goal1[0]
		start.pose.position.y = goal1[1]

		goal = PoseStamped()
		goal.header.seq = 0
		goal.header.frame_id = "map"
		goal.header.stamp = rospy.Time.now()
		goal.pose.position.x = goal2[0]
		goal.pose.position.y = goal2[1]

		start = start
		goal = goal
		tolerance = 0.5

		resp = self.get_plan(start, goal, tolerance)
		return functions.get_path_length(resp.plan)


	def send_pose_array(self):
		data = PoseArray()
		data.header.stamp = rospy.Time.now()
		data.header.frame_id = "map"
		data.poses = self.pose_array
		self.pub_pose_array.publish(data)

	
	def subscriber_pub_shutdown(self, data):
		rospy.signal_shutdown("End.")


	def subscriber_entropy(self, data):
		self.entropy = data.data
		rospy.loginfo("The actual entropy is: " + str(self.entropy))


	def subscriber_cancel(self, data):
		reverse = self.topologic_map[::-1]
		bool_closing = Bool()
		bool_closing.data = True
		self.is_reclosing = True
		self.pub_reclosing.publish(bool_closing)

		duplicate = []
		for pose in reverse:
			if pose not in duplicate:
				h_te = self.UMBRAL_ENTROPY
				for item in self.topologic_entropy[::-1]:
					if item['pose'] == pose:
						h_te = item['entropy']
						break
				
				if h_te >= self.UMBRAL_ENTROPY:
					rospy.loginfo("Robot action: Exploring Topologic Map")
					goal = self.save_goal
					goal.target_pose.pose.position.x = pose[0]
					goal.target_pose.pose.position.y = pose[1]
					goal.target_pose.pose.position.z = 0
					self.client.send_goal(goal)

					actual_pose = self.odom_pose
					actual_time = rospy.get_time()
					while functions.get_distance(actual_pose, pose) > 0.3 and rospy.get_time() - actual_time < 60:
						if actual_pose[0] != self.odom_pose[0] and actual_pose[1] != self.odom_pose[1]:
							actual_pose = self.odom_pose
				duplicate.append(pose)					

		self.pub_cancel.publish(data)
		rospy.signal_shutdown("Finish")
		rospy.loginfo("Finish.")
		sys.exit(0)


	def subscriber_feedback(self, data):
		if not self.is_reclosing:
			self.pub_feedback.publish(data)			


	def subscriber_status(self, data):
		if not self.is_reclosing:
			self.pub_status.publish(data)

	
	def subscriber_odom(self, data):
		new_pose = [data.pose.pose.position.x, data.pose.pose.position.y, 0]
		self.odom_pose = new_pose

		can_add = not self.is_reclosing

		if can_add:
			for pose in self.topologic_map:
				distance = functions.get_distance(new_pose, pose)
				is_visible = self.get_is_visible(data.pose.pose.orientation, new_pose, pose)

				can_add = distance > self.C or not is_visible

				if not can_add:
					break

		if can_add:
			self.topologic_map.append(new_pose)
			self.pose_array.append(data.pose.pose)
			self.send_pose_array()

			data_entropy = {
				"pose": new_pose,
				"entropy": self.entropy
			}

			self.topologic_entropy.append(data_entropy)


	def subscriber_scan(self, data):
		self.laser = data.ranges
		self.range_min = data.range_min
		self.range_max = data.range_max


	def subscriber_result(self, data):
		if not self.is_reclosing:
			self.pub_result.publish(data)


if __name__=='__main__':
	rospy.init_node('advanced_agent')
	explorer=Explorer()
	
	rospy.Subscriber('/odom', Odometry, explorer.subscriber_odom)
	rospy.Subscriber('/scan', LaserScan, explorer.subscriber_scan)
	
	rospy.Subscriber("/pub/pub_shutdown", Float64, explorer.subscriber_pub_shutdown)
	
	rospy.Subscriber("/move_base/status", GoalStatusArray, explorer.subscriber_status)
	rospy.Subscriber("/move_base/result", MoveBaseActionResult, explorer.subscriber_result)
	rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, explorer.subscriber_feedback)

	rospy.Subscriber("/move_base_frontier/cancel", GoalID, explorer.subscriber_cancel)

	rospy.Subscriber("/turtlebot3_slam_gmapping/entropy", Float64, explorer.subscriber_entropy)

	rospy.spin()
