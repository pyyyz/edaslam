import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import turtlebot3_env
from gym.envs.registration import register
from geometry_msgs.msg import Vector3
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String, Float64
from gazebo_msgs.msg import PerformanceMetrics
import os
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Float64, Bool
import time
from geometry_msgs.msg import PoseStamped
from collections import deque
import numpy as np
from skimage import measure
from skimage.measure import label, regionprops
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path

# mapData = OccupancyGrid()
path = Path()
path_pub = rospy.Publisher("trajectory_odom", Path, queue_size=100)
# total_length = 0.0
# global_path = Path()
last_pose_time = time.time()


class GmappingTurtleBot3WorldEnv(turtlebot3_env.TurtleBot3Env):
    def __init__(self):
        """
        This Task Env is designed for having the TurtleBot3 in the turtlebot3 world
        closed room with columns.
        It will learn how to move around without crashing.
        """
        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/turtlebot3/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="turtlebot3_gazebo",
                    launch_file_name="turtlebot3_gmapping.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/turtlebot3_gmapping/config",
                               yaml_file_name="turtlebot3_gmapping.yaml")


        # Here we will add any init functions prior to starting the MyRobotEnv
        super(GmappingTurtleBot3WorldEnv, self).__init__(ros_ws_abspath)

        # Only variable needed to be set here
        number_actions = rospy.get_param('/turtlebot3/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)


        #number_observations = rospy.get_param('/turtlebot3/n_observations')
        """
        We set the Observation space for the 6 observations
        cube_observations = [
            round(current_disk_roll_vel, 0),
            round(y_distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(y_linear_speed,1),
            round(yaw, 1),
        ]
        """

        # Actions and Observations
        self.linear_forward_speed = rospy.get_param('/turtlebot3/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/turtlebot3/linear_turn_speed')
        self.angular_speed = rospy.get_param('/turtlebot3/angular_speed')
        self.init_linear_forward_speed = rospy.get_param('/turtlebot3/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param('/turtlebot3/init_linear_turn_speed')

        self.new_ranges = rospy.get_param('/turtlebot3/new_ranges')

        self.reset_gmapping = rospy.Publisher('/reset_gmapping', String, queue_size=10)

        # We create two arrays based on the binary values that will be assigned
        # In the discretization method.
        laser_scan = self.get_laser_scan()
        self.min_range = rospy.get_param('/turtlebot3/min_range')
        self.max_laser_value = laser_scan.range_max
        self.min_laser_value = rospy.get_param('/turtlebot3/min_laser_value')
    
        num_laser_readings = 24 #5
        rospy.logwarn("MALDITASEA " + str(num_laser_readings))
        high = numpy.full((num_laser_readings), 1)
        low = numpy.full((num_laser_readings), self.min_laser_value / self.max_laser_value)

        high_coverage = numpy.full((1), 1)
        low_coverage = numpy.full((1), 0)
        # We only use two integers
        
        #self.observation_space = spaces.Dict({'laser': spaces.Box(low, high), 'entropy': spaces.Box(low_coverage, high_coverage), 'coverage': spaces.Box(low_coverage, high_coverage)})
        self.observation_space = spaces.Dict({'laser': spaces.Box(low, high), 'coverage': spaces.Box(low_coverage, high_coverage)})


        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))

        # Rewards
        self.forwards_reward = rospy.get_param("/turtlebot3/forwards_reward")
        self.turn_reward = rospy.get_param("/turtlebot3/turn_reward")
        self.end_episode_points = rospy.get_param("/turtlebot3/end_episode_points")
        self.save_action = 0

        self.pub_closing = rospy.Publisher('/custom/closing', Bool, queue_size=20)

        self.cumulated_steps = 0.0
        self.laser_filtered_pub = rospy.Publisher('/turtlebot3/laser/scan_filtered', LaserScan, queue_size=1)
        self.update_rate_real = 5
        rospy.Subscriber("/gazebo/performance_metrics", PerformanceMetrics, self.set_rate_real_time)

        self.map_coverage = 0
        self.last_coverage = 0
        rospy.Subscriber('/map', OccupancyGrid, self.subscriber_map)


        self.covariance = []
        rospy.Subscriber('/odom', Odometry, self.subscriber_odom)

        self.actual_entropy = 0
        self.last_entropy = 1
        rospy.Subscriber("/turtlebot3_slam_gmapping/entropy", Float64, self.subscriber_entropy)

        self.last_poses = deque(maxlen=10)
        self.current_time = time.time()
        self.last_time = None
        self.area = 0
        self.mapData = OccupancyGrid()
        rospy.Subscriber("/odom", Odometry, self.odom_callback2, queue_size=1)
        self.start_time0 = time.time()
        self.global_path = Path()
        self.total_length = 0.0
        self.firstcomplete = True
        self.secondcomplete = True
        self.episode_number = 1
        self.exp_ratio1 = 0.0

    def ratio(self, data):
        width = data.info.width
        height = data.info.height
        mapdata1 = data.data

        occupied_count = 0
        free_count = 0
        unknown_count = 0
        for i in range(height):
            for j in range(width):
                value = mapdata1[i * width + j]
                if value > 0:
                    occupied_count += 1
                elif value == 0:
                    free_count += 1
                else:
                    unknown_count += 1

        explored_count = occupied_count + free_count
        self.exp_ratio1 = explored_count / 16900.0          #smallsmall:45700  place3:38250 long:16900 laber91:19800 605:10700
        rospy.logwarn("ratio: " + str(self.exp_ratio1))
        return self.exp_ratio1

    def odom_callback2(self, data):
        global last_pose_time
        try:
            current_time = time.time()
            time_spend = current_time - self.start_time0
            if current_time - last_pose_time >= 1.0:
                last_pose_time = current_time
                self.global_path.header = data.header
                pose = PoseStamped()
                pose.header = data.header
                pose.pose = data.pose.pose
                self.global_path.poses.append(pose)
                path_pub.publish(self.global_path)
                if len(self.global_path.poses) >= 2:
                    prev_pose = self.global_path.poses[-2].pose
                    current_pose = self.global_path.poses[-1].pose
                    dx = current_pose.position.x - prev_pose.position.x
                    dy = current_pose.position.y - prev_pose.position.y
                    distance = math.sqrt(dx ** 2 + dy ** 2)
                    self.total_length += distance
                    exp_ratio = self.ratio(self.mapData)
                    rospy.logwarn("length: " + str(self.total_length))
                    rospy.logwarn("time_spend: " + str(time_spend))
                    episode_number = self.episode_number
                    file_name = '/home/sysu/edaslam/src/drl_agent/Result new/605/exp_path_time/long{}.txt'.format(episode_number)
                    with open(file_name, "a") as file:
                        file.write("{},{},{}\n".format(exp_ratio, self.total_length, time_spend))
                else:
                    print("not enough global_path.poses!")
        except Exception as e:
            print("An exception occurred in total_length!")

    def subscriber_odom(self, data):
        position_x = data.pose.pose.position.x
        position_y = data.pose.pose.position.y
        orientation_z = data.pose.pose.orientation.z
        orientation_w = data.pose.pose.orientation.w
        self.odom = np.array([position_x, position_y, orientation_z, orientation_w])

        self.covariance = data.pose.covariance
        pose_stamped = data.pose
        self.current_time = time.time()
        if self.last_time is None or (self.current_time - self.last_time) >= 1.0:
            self.last_poses.append(pose_stamped)
            self.last_time = self.current_time

        if len(self.last_poses) == self.last_poses.maxlen:
            if self.is_rotating_in_place():
                self._episode_done = True
                rospy.logwarn("The robot is rotating in place.")
            else:
                rospy.loginfo("The robot is moving.")

    def is_rotating_in_place(self):
        for i in range(len(self.last_poses) - 1):
            for j in range(i + 1, len(self.last_poses)):
                dx = self.last_poses[i].pose.position.x - self.last_poses[j].pose.position.x
                dy = self.last_poses[i].pose.position.y - self.last_poses[j].pose.position.y
                distance = math.sqrt(dx ** 2 + dy ** 2)
                # rospy.logwarn("distance: {:.6f} ".format(distance))
                if distance > 0.25:
                    return False
        return True

    def subscriber_map(self, data):
        self.mapData = data
        info = data.info
        rawmap = np.array(data.data)
        self.rawmap = rawmap.reshape(info.height, info.width)
        self.resolution = info.resolution
        self.p0 = np.array([info.origin.position.x, info.origin.position.y, info.origin.position.z, info.resolution])

        # neg_count = len(list(filter(lambda x: (x < 0), data.data)))    #General
        # total = len(data.data)
        # completed = 0
        # if total > 0:
        #     completed = (100 - ( neg_count * 100 / total )) / 100

        # self.area = total - neg_count
        # self.map_coverage = completed

        occupied_count = len(list(filter(lambda x: (x > 0), data.data)))        #known area
        free_count = len(list(filter(lambda x: (x == 0), data.data)))
        total = 16900.0   # laber91:19800.0 long:16900.0 small:45700
        self.area = occupied_count + free_count
        completed = self.area / total
        self.map_coverage = completed


    def subscriber_entropy(self, data):
        self.actual_entropy = data.data / 5


    def set_rate_real_time(self, data):
        if (data.real_time_factor > 0):
            self.update_rate_real = data.real_time_factor * 5
        else:
            self.update_rate_real = 5


    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base( self.init_linear_forward_speed,
                        self.init_linear_turn_speed,
                        epsilon=0.03,
                        update_rate=self.update_rate_real)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        bool_closing = Bool()
        bool_closing.data = False
        self.pub_closing.publish(bool_closing)
        self.reset_gmapping.publish(String())
        rospy.sleep(1.5)

        self.cumulated_reward = 0.0
        self.map_coverage = 0
        self.last_coverage = 0
        self.save_action = 0
        self.actual_entropy = 0
        self.last_entropy = 0
        self.covariance = []

        # Set to false Done, because its calculated asyncronously
        self._episode_done = False
        self.n_step = 0
        self.start_time = time.time()
        self.elapsed_time = 0.0

        self.last_poses = deque(maxlen=10)
        self.current_time = time.time()
        self.last_time = None
        self.area = 0

        self.start_time0 = time.time()
        self.global_path = Path()
        self.total_length = 0.0
        self.firstcomplete = True
        self.secondcomplete = True
        self.exp_ratio1 = 0.0

    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the turtlebot3
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """

        self.save_action = action
        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        if action == 0: #FORWARD
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            self.last_action = "FORWARDS"
        elif action == 1: #LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            self.last_action = "TURN_LEFT"
        elif action == 2: #RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1*self.angular_speed
            self.last_action = "TURN_RIGHT"

        # We tell TurtleBot3 the linear and angular speed to set to execute
        self.move_base(linear_speed, angular_speed, epsilon=0.03, update_rate=self.update_rate_real)
        rospy.logdebug("END Set Action ==>"+str(action))



    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        TurtleBot3Env API DOCS
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
        # We get the laser scan data
        laser_scan = self.get_laser_scan()
        # print(laser_scan)
        discretized_observations = self.discretize_scan_observation(    laser_scan,
                                                                        self.new_ranges
                                                                        )

        rospy.logdebug("Observations==>"+str(discretized_observations))
        rospy.logdebug("END Get Observation ==>")
        return discretized_observations


    def _is_done(self, observations):

        for obs in observations['laser']:
            if obs < self.min_laser_value / self.max_laser_value:
                rospy.logerr(str(obs) + "< " + str(self.min_laser_value / self.max_laser_value))
                self._episode_done = True

        if self._episode_done:
            rospy.logerr("TurtleBot3 is Too Close to wall==>")

            current_time1 = time.time()
            time_spend1 = current_time1 - self.start_time0

            file_path = '/home/sysu/edaslam/src/drl_agent/length_time.txt'
            directory = os.path.dirname(file_path)

            if not os.path.exists(directory):
                os.makedirs(directory)

            with open(file_path, 'a') as file:
                file.write('time:{}, length:{}, ratio:{}\n'.format(time_spend1, self.total_length, self.exp_ratio1))
                # file.write("time:{}  length:{}  exp_ratio:{}\n".format(time_spend1, self.total_length, self.map_coverage))

            episode_number = self.episode_number
            path_file_path = '/home/sysu/edaslam/src/drl_agent/path/global_path{}.txt'.format(episode_number)
            if not os.path.exists(os.path.dirname(path_file_path)):
                os.makedirs(os.path.dirname(path_file_path))
            with open(path_file_path, 'w') as path_file:
                for pose in self.global_path.poses:
                    path_file.write(
                        '{}, {}\n'.format(pose.pose.position.x, pose.pose.position.y))

            self.episode_number += 1

        return self._episode_done


    def _compute_reward(self, observations, done):
        reward = 0.01 #0.001
        if not done:
            obs_coverage = observations['coverage'][0]
            # rospy.logwarn("obs_coverage: {:.2f} ".format(obs_coverage))
            delta_coverage = obs_coverage - self.last_coverage
            rospy.logwarn("obs_coverage: {:.10f} ".format(obs_coverage))
            rospy.logwarn("delta_coverage: {:.10f} ".format(delta_coverage))
            if delta_coverage > 0.0001:
                reward = 0.1 + delta_coverage * 100 * obs_coverage ** 2
                self.start_time = time.time()
            else:
                self.elapsed_time = time.time() - self.start_time
                # rospy.logwarn("Elapsed Time: {:.2f} seconds".format(self.elapsed_time))
                reward = math.exp(-self.elapsed_time) * 0.001

            self.last_coverage = obs_coverage

        else:
            reward = -100

        rospy.logwarn("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward


    # Internal TaskEnv Methods

    def discretize_scan_observation(self,data,new_ranges):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """
        # self._episode_done = False
        
        discretized_ranges = []

        for i, item in enumerate(data.ranges):
            sample_interval = 360 // 24

            if i % sample_interval == 0 and i < 360:
                if item == float('Inf') or numpy.isinf(item):
                    discretized_ranges.append(1)
                elif numpy.isnan(
                        item):
                    discretized_ranges.append(round(self.min_laser_value / self.max_laser_value, 2))
                else:
                    discretized_ranges.append(
                        round(item / self.max_laser_value, 2))

                if self.min_laser_value > item > 0:
                    self._episode_done = True
                    rospy.logerr("done Validation >>> item=" + str(item) + "< " + str(self.min_range))

        self.publish_filtered_laser_scan(laser_original_data=data,
                                         new_filtered_laser_range=discretized_ranges)

        new_ranges = discretized_ranges[:]

        if self.elapsed_time > 5.0:
            self._episode_done = True

        return {"laser": new_ranges, "coverage": [self.map_coverage]}


    def publish_filtered_laser_scan(self, laser_original_data, new_filtered_laser_range):
        rospy.logdebug("new_filtered_laser_range==>"+str(new_filtered_laser_range))
        
        laser_filtered_object = LaserScan()

        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        h.frame_id = laser_original_data.header.frame_id
        
        laser_filtered_object.header = h
        laser_filtered_object.angle_min = laser_original_data.angle_min
        laser_filtered_object.angle_max = laser_original_data.angle_max
        
        new_angle_incr = abs(laser_original_data.angle_max - laser_original_data.angle_min) / len(new_filtered_laser_range)
        
        #laser_filtered_object.angle_increment = laser_original_data.angle_increment
        laser_filtered_object.angle_increment = new_angle_incr
        laser_filtered_object.time_increment = laser_original_data.time_increment
        laser_filtered_object.scan_time = laser_original_data.scan_time
        laser_filtered_object.range_min = laser_original_data.range_min
        laser_filtered_object.range_max = laser_original_data.range_max
        
        laser_filtered_object.ranges = []
        laser_filtered_object.intensities = []
        for item in new_filtered_laser_range:
            if item == 0.0:
                laser_distance = 0.1
            else:
                laser_distance = item
            laser_filtered_object.ranges.append(laser_distance)
            laser_filtered_object.intensities.append(item)
        
        self.laser_filtered_pub.publish(laser_filtered_object)
