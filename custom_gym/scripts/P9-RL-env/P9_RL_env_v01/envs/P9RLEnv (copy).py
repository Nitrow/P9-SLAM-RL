import rospy
import gym
from gym import spaces
import numpy as np
from tf import TransformListener
import os
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, geometry_msgs
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.srv import GetMap
from std_srvs.srv import Empty




def splitZones(gmap):
    zoneValue = []
    arr = np.asarray(gmap.data)
    chunks = np.array_split(arr, 16)

    for x in range(16):
        zoneValue.append(np.sum(a=chunks[x], axis=0, dtype=np.int32))

    state = zoneValue
    return state


class P9RLEnv(gym.Env):

    def __init__(self):
        self.safetyLimit = 1
        self.collisionParam = 0.2
        self.obsProximityParam = 3
        self.scan_range = []
        rospy.init_node('RLEnv', anonymous=True)
        self.tf = TransformListener()
        self.position = []
        self.quaternion = []
        self.pubAction = Twist()
        self.reward = 0
        self.data = []
        self.state = []
        self.data2 = []
        self.action = []
        self.rewardMapOld = 0
        self.done = False
        self.TimeoutCounter = 0
        self.lidarDiscretization = 10

        self.pub = rospy.Publisher('/vehicle_blue/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('/syscommand', String, queue_size=1)
        rospy.wait_for_service('/dynamic_map')
        self.getmap = rospy.ServiceProxy('/dynamic_map', GetMap, persistent=True)
        self.maxAngSpeed = 1
        self.maxLinSpeed = 0.2

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        self.action_space = spaces.Box(low=np.array([0, -self.maxAngSpeed]),
                                       high=np.array([self.maxLinSpeed, self.maxAngSpeed]), dtype=np.float16)
        self.observation_space = spaces.Box(low=-1, high=100, shape=(30,), dtype=np.float16)

    def reset(self):
        # RESET ENVIRONMENT

        self.TimeoutCounter = 0

        if self.done is True:
            rospy.wait_for_service('gazebo/reset_simulation')

            try:
                self.reset_proxy()
            except rospy.ServiceException as e:
                print("gazebo/reset_simulation service call failed")

            self.pub2.publish("reset")

            self.done = False

        self.unpause_proxy()
        time.sleep(1)
        scan = rospy.wait_for_message("/scan", LaserScan, timeout=1000)
        gmap = rospy.wait_for_message("/map", OccupancyGrid, timeout=1000)

        self.pause_proxy()
        self.getPose()
        self.scan_range, minScan = self.scanRange(scan)

        self.rewardMapOld = np.sum(a=splitZones(gmap), axis=0, dtype=np.int64)

        state = splitZones(gmap) + self.scan_range + [self.position[0], + self.position[1], self.quaternion[2],
                                                      self.quaternion[3]]
        return state

    def step(self, action):

        self.TimeoutCounter += 1

        linear_vel = action[0]
        ang_vel = action[1]

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel
        vel_cmd.angular.z = ang_vel
        self.pub.publish(vel_cmd)

        self.unpause_proxy()

        scan = rospy.wait_for_message("/scan", LaserScan, timeout=1000)
        gmap = rospy.wait_for_message("/map", OccupancyGrid, timeout=1000)

        self.pause_proxy()






        state, self.done = self.setStateAndDone(gmap, scan)
        reward = self.setReward(state, self.done)
        return [state, reward, self.done, {}]

    def setReward(self, state, done):
        self.rewardMap = np.sum(a=state, axis=0, dtype=np.int64)
        self.reward = self.rewardMap - self.rewardMapOld
        self.rewardMapOld = self.rewardMap

        self.reward += self.rewardObstacleProximity()
        self.reward += -1
        if done:
            self.reward += 1000
        return self.reward

    def render(self, mode='human'):
        pass

    def setStateAndDone(self, gmap, scan):

        # # Set state and done

        done = False
        self.getPose()
        self.scan_range, minScan = self.scanRange(scan)
        if minScan < self.collisionParam or self.TimeoutCounter == 100000:
            done = True
        state = splitZones(gmap) + self.scan_range + [self.position[0], + self.position[1], self.quaternion[2],
                                                      self.quaternion[3]]

        return state, done

    def getPose(self):
        if self.tf.frameExists("vehicle_blue/base_link") and self.tf.frameExists("vehicle_blue/odom"):
            t = self.tf.getLatestCommonTime("vehicle_blue/base_link", "vehicle_blue/odom")
            self.position, self.quaternion = self.tf.lookupTransform("vehicle_blue/base_link", "vehicle_blue/odom", t)
        return self.position, self.quaternion

    def rewardObstacleProximity(self):
        closestObstacle = min(self.scan_range)
        if closestObstacle <= self.safetyLimit:
            return self.obsProximityParam * -(1 - (closestObstacle / self.safetyLimit))
        else:
            return 0
    def scanRange(self, scan):
    #
        scan_range = []

        for x in range(0, 100, self.lidarDiscretization):
            end = x + self.lidarDiscretization
            scan_range.append(min(scan.ranges[x:end]))

        minScan = min(list(filter(lambda a: a != 0, scan_range[:])))
        return scan_range, minScan
