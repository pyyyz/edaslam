#!/usr/bin/env python3

import os
import sys
import rospy
import functions
import matplotlib.pyplot as plt

from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float64, Bool, Empty as EmptyMsg
from geometry_msgs.msg import PoseStamped


class Plot:

	def __init__(self):
		#Publishers
		self.pub_path = rospy.Publisher('/pub/path', Path, queue_size=100)
		self.pub_reset = rospy.Publisher('/reset', EmptyMsg, queue_size=100)
		self.pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=100)
		self.pub_umbral = rospy.Publisher('/pub/umbral', Float64, queue_size=20)
		self.pub_shutdown = rospy.Publisher('/pub/pub_shutdown', Float64, queue_size=20)

		self.path = Path()
		
		self.umbral = 4.567
		self.map_coverage = 0
		self.is_closing = False
		self.initial_time = None
		self.is_reclosing = False
		
		self.time = []
		self.entropy = []
		self.time_closing = []
		self.entropy_closing = []
		self.time_reclosing = []
		self.entropy_reclosing = []


	def subscriber_map(self, data):
		neg_count = len(list(filter(lambda x: (x < 0), data.data))) 
		total = len(data.data)
		completed = 0
		if total > 0:
			completed = 100 - ( neg_count * 100 / total )
		self.map_coverage = completed


	def subscriber_entropy(self, data):
		diff = 0

		if self.initial_time is None:
			diff = 0
			self.initial_time = rospy.get_time()
		else:
			now = rospy.get_time()
			diff = now - self.initial_time

		if not self.is_reclosing:
			if self.is_closing:
				self.time_closing.append(int(diff))
				self.entropy_closing.append(data.data)
			else:
				self.time.append(int(diff))
				self.entropy.append(data.data)
		else:
			self.time_reclosing.append(int(diff))
			self.entropy_reclosing.append(data.data)


	def subscriber_odom(self, data):
		self.path.header = data.header
		pose = PoseStamped()
		pose.header = data.header
		pose.pose = data.pose.pose
		self.path.poses.append(pose)
		self.pub_path.publish(self.path)


	def subscriber_closing(self, data):
		self.is_closing = data.data


	def subscriber_reclosing(self, data):
		self.is_reclosing = data.data

	def aux_cancel(self):
		os.system('rosrun map_server map_saver -f /home/sysu/edaslam/src/drl_agent/tests/map_'+str(self.umbral)+'_'+str(rospy.Time.now()))

		#Distancia Recorrida
		distance = functions.get_path_length(self.path)
		path_text = "Path Length (m): " + str(distance)

		#Cubrimiento del Mapa
		coverage_text = "Map Coverage (%): " + str(self.map_coverage)

		#Entropia Promedio
		entropy = 0
		if (len(self.entropy) + len(self.entropy_closing) + len(self.entropy_reclosing) > 0):
			entropy = (sum(self.entropy) + sum(self.entropy_closing) + sum(self.entropy_reclosing)) / (len(self.entropy) + len(self.entropy_closing) + len(self.entropy_reclosing))
		entropy_text = "AVG Entropy: " + str(entropy)

		#Tiempo Total
		now = rospy.get_time()
		total_time = now - self.initial_time
		time_text = "Total Time (s): " + str(total_time)

		plt.figure()
		plt.axis(ymax=5.2)
		plt.plot(self.time, self.entropy, '-', label="Entropy", color='black')
		#plt.plot(self.time_closing, self.entropy_closing, 'ro', label="Loop-Closing", color='b')
		#plt.plot(self.time_reclosing, self.entropy_reclosing, 'ro', label="Exploring Topologic Map", color='r')
		plt.plot([], [], ' ', label=entropy_text)
		plt.plot([], [], ' ', label=path_text)
		#plt.plot([], [], ' ', label=coverage_text)
		plt.plot([], [], ' ', label=time_text)
		plt.xlabel('Time')
		plt.ylabel('Entropy')
		plt.grid(True)
		plt.legend()

		#plt.savefig('/home/sysu/edaslam/src/advanced_agent/simulations/plot/plot_entropy_'+str(self.umbral)+'_'+str(rospy.Time.now())+'.png')
		plt.savefig('/home/sysu/edaslam/src/drl_agent/tests/plot_entropy_'+str(self.umbral)+'_'+str(rospy.Time.now())+'.png')
		#os.system('rosrun map_server map_saver -f /home/sysu/edaslam/src/advanced_agent/simulations/map/map_'+str(self.umbral)+'_'+str(rospy.Time.now()))
		
		#rospy.signal_shutdown("End.")
		#os.system('rosnode kill -a')

		self.path = Path()
		
		self.umbral = 4.567
		self.map_coverage = 0
		self.is_closing = False
		self.initial_time = now
		self.is_reclosing = False
		
		self.time = []
		self.entropy = []
		self.time_closing = []
		self.entropy_closing = []
		self.time_reclosing = []
		self.entropy_reclosing = []
	
	def subscriber_cancel(self, data=None):
		self.aux_cancel()


if __name__=='__main__':
	rospy.init_node('plot_data')
	plot=Plot()
	
	rospy.Subscriber("/odom", Odometry, plot.subscriber_odom)
	rospy.Subscriber('/map', OccupancyGrid, plot.subscriber_map)
	#rospy.Subscriber("/gazebo/reset_simulation", Empty, plot.subscriber_cancel)
	#rospy.Subscriber("/move_base/cancel", GoalID, plot.subscriber_cancel)

	#rospy.Subscriber("/custom/closing", Bool, plot.subscriber_closing)
	rospy.Subscriber("/custom/closing", Bool, plot.subscriber_cancel)
	rospy.Subscriber("/custom/reclosing", Bool, plot.subscriber_reclosing)

	rospy.Subscriber("/turtlebot3_slam_gmapping/entropy", Float64, plot.subscriber_entropy)

	rospy.spin()

	plot.aux_cancel()