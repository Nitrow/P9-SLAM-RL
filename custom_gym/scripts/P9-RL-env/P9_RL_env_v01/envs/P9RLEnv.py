import rospy
import gym
from gym import spaces
import numpy as np
from custom_gym.srv import StepFunction
from tf import TransformListener

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def scanRange(scan):
    #
    scan_range = []
    for i in range(len(scan.ranges)):
        if scan.ranges[i] == float('Inf'):
            scan_range.append(5)
        elif np.isnan(scan.ranges[i]):
            scan_range.append(0)
        else:
            scan_range.append(scan.ranges[i])

    minScan = min(list(filter(lambda a: a != 0, scan_range[:])))
    return scan_range, minScan


def splitZones(gmap):

    zoneValue = []
    arr = np.asarray(gmap.data)
    chunks = np.array_split(arr, 16)

    for x in range(16):
        zoneValue.append(np.sum(a=chunks[x], axis=0, dtype=np.int8))

    state = zoneValue
    return state


class P9RLEnv(gym.Env):

    def __init__(self):
        self.safetyLimit = 0.2
        self.obsProximityParam = 1
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
        self.pub = rospy.Publisher('/model/vehicle_blue/cmd_vel', Twist, queue_size=10)
        #rospy.wait_for_service('/stepper')
        #self.stepper = rospy.ServiceProxy('/stepper', StepFunction, persistent=True)
        self.maxAngSpeed = 1
        self.maxLinSpeed = 0.2

        self.action_space = spaces.Box(low=np.array([0, -self.maxAngSpeed]),
                                       high=np.array([self.maxLinSpeed, self.maxAngSpeed]), dtype=np.float16)
        self.observation_space = spaces.Box(low=-1, high=100, shape=(123,), dtype=np.float16)

    def reset(self):
        # RESET ENVIRONMENT

        scan = rospy.wait_for_message("/lidar", LaserScan, timeout=5)
        gmap = rospy.wait_for_message("/map", OccupancyGrid, timeout=5)

        self.getPose()
        self.scan_range, minScan = scanRange(scan)

        state = splitZones(gmap) + self.scan_range + self.position + self.quaternion
        return state

    def step(self, action):
        self.pubAction.linear.x = action[0]
        self.pubAction.angular.z = action[1]
        self.pub.publish(self.pubAction)

        # self.stepper(1)

        scan = None
        gmap = None
        while scan and gmap is None:
            try:
                scan = rospy.wait_for_message("/lidar", LaserScan, timeout=1000)
                gmap = rospy.wait_for_message("/map", OccupancyGrid, timeout=5)
            except:
                pass

        state, done = self.setStateAndDone(gmap, scan)
        reward = self.setReward(state, done)
        return [state, reward, done, {}]

    def setReward(self, state, done):
        self.reward = np.sum(a=state, axis=0, dtype=np.int8)
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
        self.scan_range, minScan = scanRange(scan)
        if minScan < self.safetyLimit:
            done = True
        # DIVIDE STATE INTO ZONES????????
        state = splitZones(gmap) + self.scan_range + self.position + self.quaternion

        return state, done

    def getPose(self):
        if self.tf.frameExists("base_link") and self.tf.frameExists("vehicle_blue/odom"):
            t = self.tf.getLatestCommonTime("base_link", "vehicle_blue/odom")
            self.position, self.quaternion = self.tf.lookupTransform("base_link", "vehicle_blue/odom", t)
        return self.position, self.quaternion

    def rewardObstacleProximity(self):

        closestObstacle = min(self.scan_range)
        if closestObstacle <= self.safetyLimit:
            return self.obsProximityParam * -(1 - (closestObstacle / self.safetyLimit))
        else:
            return 0
