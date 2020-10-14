#!/usr/bin/env python
import rospy
import gym
from gym import spaces
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist


class P9RLEnv(gym.Env):

    def __init__(self):
        rospy.init_node('RLEnv', anonymous=True)
        self.pubAction = Twist()
        self.state = []
        self.action = [0, 0]
        self.reward = 0
        self.done = False
        self.reward = 0
        self.sub = rospy.Subscriber("/map", OccupancyGrid, self.callback)
        self.pub = rospy.Publisher('action', Twist, queue_size=10)
        self.maxAngSpeed = 10
        self.maxLinSpeed = 10

        self.action_space = spaces.Box(low=np.array([-self.maxLinSpeed, -self.maxAngSpeed]),
                                       high=np.array([self.maxLinSpeed, self.maxAngSpeed]), dtype=np.float16)
        self.observation_space = spaces.Box(low=-1, high=100, shape=(100000,), dtype=np.int8)
        rospy.spin()

    def callback(self, data):
        self.state = data.data

    def reset(self):
        # RESET ENVIRONMENT
        return np.asarray(self.state)

    def step(self, action):
        self.action = action
        self.pubAction.linear.x = self.action[0]
        self.pubAction.angular.z = self.action[1]
        self.pub.publish(self.pubAction.linear.x, self.pubAction.angular.z)
        # TAKE STEP
        # GET REWARD
        self.reward = self.setReward()

        return [self.state, self.reward, self.done]

    def setReward(self):
        return self.reward, self.done
