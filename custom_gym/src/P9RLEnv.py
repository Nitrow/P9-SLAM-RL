#!/usr/bin/env python3
import os
import rospy
import gym
from gym import spaces
import numpy as np
import



from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class P9RLEnv(gym.Env):

    def __init__(self):
        rospy.init_node('RLEnv', anonymous=True)
        self.pubAction = Twist()
        self.action = []
        self.pub = rospy.Publisher('action', Twist, queue_size=10)
        self.maxAngSpeed = 1
        self.maxLinSpeed = 1

        self.action_space = spaces.Box(low=np.array([0, -self.maxAngSpeed]),
                                       high=np.array([self.maxLinSpeed, self.maxAngSpeed]), dtype=np.float16)
        self.observation_space = spaces.Box(low=-1, high=100, shape=(100000000000,), dtype=np.int8), spaces.Box(low=0, high=10, shape=(360,), dtype=np.float16)
        rospy.spin()

    def reset(self):
        # RESET ENVIRONMENT
        state = 1
        return state

    def step(self, action):
        self.action = action
        self.pubAction.linear.x = self.action[0]
        self.pubAction.angular.z = self.action[1]
        self.pub.publish(self.pubAction)

        os.system("ign service -r -i -s /world/diff_drive/control --reqtype ignition.msgs.WorldControl --reptype "
                  "ignition.msgs.Boolean --timeout 1000 --req 'pause: true, multi_step: 1'")


        data = None
        data2 = None
        while data and data2 is None:
            try:
                data = rospy.wait_for_message("/map", OccupancyGrid, timeout=5)
                data2 = rospy.wait_for_message("/lidar", LaserScan, timeout=5)
            except:
                pass

        # Get reward
        state, done = self.setStateAndDone(data, data2)
        reward = self.setReward(state, done)

        return [state, reward, done]

    def setReward(self, state, done):
        reward = np.sum(a=state, axis=0, dtype=np.int8)
        reward += -1
        if done:
            reward += 10000000000
        return reward

    def render(self, mode='human'):
        pass

    def setStateAndDone(self, data, data2):
        state = data, data2
        done = 1
        # Set state and done
        return state, done
