#!/usr/bin/env python3

import time
import numpy as np
import rospkg
# ROS packages required
import rospy
from gym import wrappers
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment
from stable_baselines3 import PPO
import matplotlib.pyplot as plt
from realcar_env import RealCar

def main():

    # loadModel = False
    # saveModel = True
    # continueTraining = True

    # loadModel = False
    # saveModel = True
    # continueTraining = False

    #test
    loadModel = True
    saveModel = False
    continueTraining = False

    car, modelPath = init()

    if (loadModel):
        rospy.logwarn("Loading Model...")
        model = PPO.load(modelPath)
        inited = False
    else:
        if (continueTraining):
            rospy.logwarn("Continue training")
            model = PPO.load(modelPath, car)
        else:
            model = PPO('MultiInputPolicy', car, verbose=1)
      
        model.learn(total_timesteps=3500000)
        rospy.logwarn("Training finished")
        inited = True

        if (saveModel):
            rospy.logwarn("Saving Model...")
            model.save(modelPath)
            rospy.logwarn("Model saved")


    rospy.logwarn("Start prediction...")
    evaluate(model, car, inited)


def evaluate(model, car: RealCar, inited, num_episodes=5000):
        """
        Evaluate a RL agent
        :param model: (BaseRLModel object) the RL Agent
        :param num_episodes: (int) number of episodes to evaluate it
        :return: (float) Mean reward for the last num_episodes
        """
        all_episode_rewards = []
        obs = car._get_obs()
        while True:
            if car._episode_done:
                car.move_base(-0.30, 0.0, epsilon=0.0, update_rate=car.update_rate_real)
                rospy.logwarn("RECOVERY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                obs = car._get_obs()
            else:
                action, _states = model.predict(obs, deterministic=True)
                obs, reward, done, info = car.step(action)

def init():
    rospy.init_node('test_realcar',
                    anonymous=True, log_level=rospy.WARN)
    # task_and_robot_environment_name = rospy.get_param(
    #     '/turtlebot3/task_and_robot_environment_name')
    # env = StartOpenAI_ROS_Environment(
    #     task_and_robot_environment_name)
    car = RealCar()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('drl_agent')
    outdir = pkg_path + '/training_results'
    modelPath = outdir + "/large"
    # car = wrappers.Monitor(car, outdir, force=True)
    return car, modelPath

if __name__ == '__main__':
    main()
