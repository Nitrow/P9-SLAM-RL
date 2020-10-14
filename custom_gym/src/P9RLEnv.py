#!/usr/bin/env python
import rospy
import gym
from gym import spaces
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist


class P9RLEnv(gym.Env):

    def __init__(self):
        self.sub = rospy.Subscriber("/map", OccupancyGrid, self.callback)
        self.pub = rospy.Publisher('action', Twist, queue_size=10)
        self.state = []
        self.mapdata = []
        self.maxAngSpeed = 10  # 2.84 max
        self.maxLinSpeed = 10  # 0.22 max

        self.action_space = spaces.Box(low=np.array([-self.maxLinSpeed, -self.maxAngSpeed]),
                                       high=np.array([0, self.maxAngSpeed]), dtype=np.float16)
        self.observation_space = spaces.Box(low=-1, high=100, shape=(100000,), dtype=np.int8)

    def callback(self, data):
        self.mapdata = data.data

    def listener(self):
        rospy.init_node('RLenv', anonymous=True)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def reset(self):
        # RESET ENVIRONMENT
        self.state = self.mapdata

        return np.asarray(self.state)

    def step(self, action):
        test2 = 2
        self.pub.publish(action)
        # TAKE STEP
        self.state = self.mapdata
        # GET REWARD

        return [self.state, self.reward, self.done]
