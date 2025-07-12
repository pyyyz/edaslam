import rospy
import gym
from gym.utils import seeding
from .gazebo_connection import GazeboConnection
from .controllers_connection import ControllersConnection
#https://bitbucket.org/theconstructcore/theconstruct_msgs/src/master/msg/RLExperimentInfo.msg
from openai_ros.msg import RLExperimentInfo

from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import os
import ast
import time
import cv2
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Point
import heapq
import math
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import concurrent.futures

mapData1 = OccupancyGrid()
def mapCallBack(data):
    global mapData1
    mapData1 = data

def save_map(ep_num):
    map_width = mapData1.info.width
    map_height = mapData1.info.height
    map_resolution = mapData1.info.resolution
    map_array = mapData1.data
    map_array = np.flipud(map_array)
    image = Image.new("L", (map_width, map_height))
    valid_grid_count = 0

    for i in range(map_height):
        for j in range(map_width):
            index = i * map_width + j
            value = map_array[index]

            if value == -1:
                grayscale_value = 127
            elif value == 100:
                grayscale_value = 0
                valid_grid_count += 1
            else:
                grayscale_value = 255
                valid_grid_count += 1

            image.putpixel((j, i), grayscale_value)

    # Save the image as a .pgm file
    map_pgm_filename = "/home/sysu/edaslam/src/drl_agent/map/map_{}.png".format(ep_num)
    image.save(map_pgm_filename)
    # Save map metadata as .yaml file
    map_yaml_filename = "/home/sysu/edaslam/src/drl_agent/map/map_{}.yaml".format(ep_num)
    with open(map_yaml_filename, "w") as f:
        f.write("image: " + map_pgm_filename + "\n")
        f.write("map_width: {}\n".format(map_width))
        f.write("map_height: {}\n".format(map_height))
        f.write("resolution: {}\n".format(map_resolution))
        f.write("origin: [0.0, 0.0, 0.0]\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
        f.write("mode: raw\n")
        f.write("valid_grid_count: {}\n".format(valid_grid_count))
    print("Map saved successfully!")



# https://github.com/openai/gym/blob/master/gym/core.py
class RobotGazeboEnv(gym.Env):

    def __init__(self, robot_name_space, controllers_list, reset_controls, start_init_physics_parameters=True, reset_world_or_sim="SIMULATION"):

        # To reset Simulations
        rospy.logdebug("START init RobotGazeboEnv")
        self.gazebo = GazeboConnection(start_init_physics_parameters,reset_world_or_sim)
        self.controllers_object = ControllersConnection(namespace=robot_name_space, controllers_list=controllers_list)
        self.reset_controls = reset_controls
        self.seed()

        # Set up ROS related variables
        self.episode_num = 0
        self.cumulated_episode_reward = 0
        self.reward_pub = rospy.Publisher('/openai/reward', RLExperimentInfo, queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, mapCallBack)
        # We Unpause the simulation and reset the controllers if needed
        """
        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.
        """
        self.gazebo.unpauseSim()
        if self.reset_controls:
            self.controllers_object.reset_controllers()

        rospy.logdebug("END init RobotGazeboEnv")

        self.all_episode_rewards = []
        self.first_call = True
        # self.load_episode_rewards()

        self.time_file_path = '/home/sysu/edaslam/src/drl_agent/training_results/time_rewards.txt'
        self.time_records_file = '/home/sysu/edaslam/src/drl_agent/training_results/training_start_time.txt'

        directory = os.path.dirname(self.time_file_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        self._initialize_training_metadata()


        self.inflation_radius = 0.1
        self.reachable_frontiers = []
        self.frontier_check_interval = 2.0
        self.last_frontier_check_time = rospy.Time.now()
        self.no_reachable_frontiers = False
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_pose = None
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None

    def _initialize_training_metadata(self):
        if os.path.exists(self.time_records_file):
            with open(self.time_records_file, 'r') as f:
                try:
                    self.training_start_time = float(f.read().strip())
                    rospy.loginfo(f"start time: {self.training_start_time}")
                except:
                    rospy.logwarn("new time")
                    self._create_new_start_time()
        else:
            self._create_new_start_time()

        self.episode_num = 0
        self.cumulated_episode_reward = 0
        self.last_episode_end_time = self.training_start_time

    def _create_new_start_time(self):
        self.training_start_time = time.time()
        rospy.loginfo(f"new time: {self.training_start_time}")
        with open(self.time_records_file, 'w') as f:
            f.write(str(self.training_start_time))


    def load_episode_rewards(self):
        file_path = '/home/sysu/edaslam/src/drl_agent/training_results/episode_rewards.txt'
        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                lines = file.readlines()
                self.all_episode_rewards = [float(line.strip()) for line in lines]
    def plot_reward(self, reward):
        save_path = '/home/sysu/edaslam/src/drl_agent/picture/episode_reward.png'
        plt.ion()
        fig, ax = plt.subplots()
        line, = ax.plot([], [], 'r-')
        if reward != 0:
            self.all_episode_rewards.append(reward + 100.0)
            line.set_xdata(range(len(self.all_episode_rewards)))
            line.set_ydata(self.all_episode_rewards)
            ax.set_xlim([0, len(self.all_episode_rewards)])
            max_abs_reward = max(np.abs(np.array(self.all_episode_rewards)))
            ax.set_ylim([0, max_abs_reward * 1.1])
            # ax.set_ylim([0, max(self.all_episode_rewards) * 1.1])
            ax.set_xlabel('Episode')
            ax.set_ylabel('Cumulative Reward')
            ax.set_title('Episode Rewards over Episodes')
            plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
            plt.clf()

            file_path = '/home/sysu/edaslam/src/drl_agent/training_results/episode_rewards.txt'
            directory = os.path.dirname(file_path)

            if not os.path.exists(directory):
                os.makedirs(directory)
            rewards_str = '\n'.join(map(str, self.all_episode_rewards))

            with open(file_path, 'w') as file:
                file.write(rewards_str + "\n")

        plt.ioff()

    # Env methods
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def odom_callback(self, msg):
        self.current_pose = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            },
            'stamp': msg.header.stamp
        }
        rospy.logdebug(f"Updated position from odom: x={self.current_pose['position']['x']:.3f}, y={self.current_pose['position']['y']:.3f}")

    def get_robot_position(self):
        if self.current_pose is None:
            rospy.logwarn("Robot position not available from odom yet")
            return None

        return self.current_pose

    def world_to_map(self, x, y):
        if not self.map_resolution or not self.map_origin:
            return None

        mx = int((x - self.map_origin.position.x) / self.map_resolution)
        my = int((y - self.map_origin.position.y) / self.map_resolution)
        return mx, my


    def map_to_world(self, mx, my):
        if not self.map_resolution or not self.map_origin:
            return None

        x = self.map_origin.position.x + mx * self.map_resolution
        y = self.map_origin.position.y + my * self.map_resolution
        return x, y

    def is_valid_cell(self, x, y):
        if not self.map_data or x < 0 or y < 0 or x >= self.map_width or y >= self.map_height:
            return False

        index = y * self.map_width + x
        cell_value = self.map_data[index]

        return cell_value == 0

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0),
                     (1, 1), (1, -1), (-1, 1), (-1, -1)]

        max_iterations = self.map_width * self.map_height // 2
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            current = heapq.heappop(open_set)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.is_valid_cell(neighbor[0], neighbor[1]):
                    continue

                diagonal = 1 if abs(dx) + abs(dy) == 2 else 0
                tentative_g = g_score[current] + (math.sqrt(2) if diagonal else 1)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)

                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def _detect_frontiers(self):
        if self.map_data is None:
            return []

        free_mask = []
        for i in range(self.map_height):
            row = []
            for j in range(self.map_width):
                idx = i * self.map_width + j
                row.append(1 if self.map_data[idx] == 0 else 0)
            free_mask.append(row)

        min_x, min_y = self.map_width, self.map_height
        max_x, max_y = 0, 0

        for i in range(self.map_height):
            for j in range(self.map_width):
                if free_mask[i][j] == 1:
                    if j < min_x: min_x = j
                    if j > max_x: max_x = j
                    if i < min_y: min_y = i
                    if i > max_y: max_y = i

        pad = 10
        min_x = max(0, min_x - pad)
        min_y = max(0, min_y - pad)
        max_x = min(self.map_width, max_x + pad)
        max_y = min(self.map_height, max_y + pad)

        frontier_points = []

        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]

        for i in range(min_y, max_y):
            for j in range(min_x, max_x):
                if free_mask[i][j] != 1:
                    continue
                has_unknown_neighbor = False
                for dx, dy in neighbors:
                    ni, nj = i + dy, j + dx
                    if 0 <= ni < self.map_height and 0 <= nj < self.map_width:
                        idx = ni * self.map_width + nj
                        if self.map_data[idx] == -1:
                            has_unknown_neighbor = True
                            break

                if has_unknown_neighbor:
                    wx, wy = self.map_to_world(j, i)
                    frontier_points.append((wx, wy))

        if len(frontier_points) > 1000:
            grid_size = 0.5
            grid_buckets = {}

            for point in frontier_points:
                x, y = point
                grid_x = int(x / grid_size)
                grid_y = int(y / grid_size)
                bucket_key = (grid_x, grid_y)

                if bucket_key not in grid_buckets:
                    grid_buckets[bucket_key] = []
                grid_buckets[bucket_key].append(point)

            sparse_points = []
            for bucket_key, points_in_bucket in grid_buckets.items():
                if points_in_bucket:
                    center_x = sum(p[0] for p in points_in_bucket) / len(points_in_bucket)
                    center_y = sum(p[1] for p in points_in_bucket) / len(points_in_bucket)

                    min_dist = float('inf')
                    best_point = None
                    for point in points_in_bucket:
                        dist = (point[0] - center_x) ** 2 + (point[1] - center_y) ** 2
                        if dist < min_dist:
                            min_dist = dist
                            best_point = point

                    sparse_points.append(best_point)

            frontier_points = sparse_points

        return frontier_points

    def _check_reachable_frontiers(self):
        current_time = rospy.Time.now()
        if (current_time - self.last_frontier_check_time).to_sec() < 5.0:
            return

        self.last_frontier_check_time = current_time

        robot_pose = self.get_robot_position()
        if not robot_pose:
            return

        if hasattr(self, 'last_robot_position'):
            dx = robot_pose['position']['x'] - self.last_robot_position['x']
            dy = robot_pose['position']['y'] - self.last_robot_position['y']
            if math.sqrt(dx * dx + dy * dy) < 0.5:  # 移动小于0.5米时使用缓存
                return

        self.last_robot_position = robot_pose['position']

        global mapData1
        if not mapData1:
            return

        self.map_data = mapData1.data
        self.map_resolution = mapData1.info.resolution
        self.map_origin = mapData1.info.origin
        self.map_width = mapData1.info.width
        self.map_height = mapData1.info.height

        frontier_points = self._detect_frontiers()

        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            futures = []
            for point in frontier_points:
                mx, my = self.world_to_map(point[0], point[1])
                if mx is not None and my is not None:
                    futures.append(executor.submit(
                        self._is_reachable,
                        self.world_to_map(robot_pose['position']['x'], robot_pose['position']['y']),
                        (mx, my)
                    ))

            reachable_frontiers = []
            for future, point in zip(concurrent.futures.as_completed(futures), frontier_points):
                if future.result():
                    reachable_frontiers.append(point)

        self.reachable_frontiers = reachable_frontiers
        self.no_reachable_frontiers = len(reachable_frontiers) == 0

        if hasattr(self, 'last_frontiers') and set(frontier_points) == set(self.last_frontiers):
            return

        self.last_frontiers = frontier_points


    def step(self, action):
        """
        Function executed each time step.
        Here we get the action execute it in a time step and retrieve the
        observations generated by that action.
        :param action:
        :return: obs, reward, done, info
        """

        """
        Here we should convert the action num to movement action, execute the action in the
        simulation and get the observations result of performing that action.
        """
        rospy.logdebug("START STEP OpenAIROS")

        current_time = rospy.Time.now()
        if (current_time - self.last_frontier_check_time).to_sec() > self.frontier_check_interval:
            self._check_reachable_frontiers()
            self.last_frontier_check_time = current_time

        self.gazebo.unpauseSim()
        self._set_action(action)
        self.gazebo.pauseSim()
        obs = self._get_obs()
        # done = self._is_done(obs)
        done = self._is_done(obs) or self.no_reachable_frontiers
        info = {}
        reward = self._compute_reward(obs, done)
        self.cumulated_episode_reward += reward

        rospy.logdebug("END STEP OpenAIROS")

        return obs, reward, done, info

    def getObs(self):
        return self._get_obs()

    def reset(self):
        rospy.logdebug("Reseting RobotGazeboEnvironment")
        self._reset_sim()
        self._init_env_variables()
        self._update_episode()
        obs = self._get_obs()

        self.reachable_frontiers = []
        self.last_frontier_check_time = rospy.Time.now()
        self.no_reachable_frontiers = False

        rospy.logdebug("END Reseting RobotGazeboEnvironment")
        return obs

    def close(self):
        """
        Function executed when closing the environment.
        Use it for closing GUIS and other systems that need closing.
        :return:
        """
        rospy.logdebug("Closing RobotGazeboEnvironment")
        rospy.signal_shutdown("Closing RobotGazeboEnvironment")

    def _update_episode(self):
        """
        Publishes the cumulated reward of the episode and
        increases the episode number by one.
        :return:
        """
        #rospy.logwarn("PUBLISHING REWARD...")
        self._publish_reward_topic(
                                    self.cumulated_episode_reward,
                                    self.episode_num
                                    )
        rospy.logwarn("PUBLISHING REWARD...DONE="+str(self.cumulated_episode_reward)+",EP="+str(self.episode_num))

        current_time = time.time()

        episode_duration = current_time - self.last_episode_end_time
        rospy.loginfo(f"episode {self.episode_num} time_duration: {episode_duration:.3f} ")

        self._save_cumulative_time(current_time)

        self.last_episode_end_time = current_time

        save_map(self.episode_num)
        self.plot_reward(self.cumulated_episode_reward)

        self.episode_num += 1
        self.cumulated_episode_reward = 0

    def _save_cumulative_time(self, current_time):
        try:
            cumulative_time = current_time - self.training_start_time
            # 记录数据
            time_line = f"{cumulative_time:.3f}\n"
            with open(self.time_file_path, 'a') as time_file:
                time_file.write(time_line)
            rospy.loginfo(f"episode end,cumulative_time: {cumulative_time:.3f}秒")
        except Exception as e:
            rospy.logerr(f"Failed to save time: {str(e)}")

    def _publish_reward_topic(self, reward, episode_number=1):
        """
        This function publishes the given reward in the reward topic for
        easy access from ROS infrastructure.
        :param reward:
        :param episode_number:
        :return:
        """
        reward_msg = RLExperimentInfo()
        reward_msg.episode_number = episode_number
        reward_msg.episode_reward = reward
        self.reward_pub.publish(reward_msg)

    # Extension methods
    # ----------------------------

    def _reset_sim(self):
        """Resets a simulation
        """
        rospy.logdebug("RESET SIM START")
        if self.reset_controls :
            rospy.logdebug("RESET CONTROLLERS")
            self.gazebo.unpauseSim()
            self.controllers_object.reset_controllers()
            self._check_all_systems_ready()
            self._set_init_pose()
            self.gazebo.pauseSim()
            self.gazebo.resetSim()
            self.gazebo.unpauseSim()
            self.controllers_object.reset_controllers()
            self._check_all_systems_ready()
            self.gazebo.pauseSim()

        else:
            rospy.logwarn("DONT RESET CONTROLLERS")
            self.gazebo.unpauseSim()
            self._check_all_systems_ready()
            self._set_init_pose()
            self.gazebo.pauseSim()
            self.gazebo.resetSim()
            self.gazebo.unpauseSim()
            self._check_all_systems_ready()
            self.gazebo.pauseSim()

        rospy.logdebug("RESET SIM END")
        return True

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        raise NotImplementedError()

    def _get_obs(self):
        """Returns the observation.
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _is_done(self, observations):
        """Indicates whether or not the episode is done ( the robot has fallen for example).
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _env_setup(self, initial_qpos):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.
        """
        raise NotImplementedError()

