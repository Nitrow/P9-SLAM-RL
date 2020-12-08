import rospy
import gym
from gym import spaces
import numpy as np
from tf import TransformListener
import os
from nav_msgs.msg import Odometry
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, geometry_msgs
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.srv import GetMap
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion







class P9RLEnv(gym.Env):

    def __init__(self):
        self.safetyLimit = 1
        self.collisionParam = 0.5
        self.obsProximityParam = 5
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
        self.lidarDiscretization = 30

        self.pub = rospy.Publisher('/vehicle_blue/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('/syscommand', String, queue_size=1)
        self.maxAngSpeed = 1
        self.maxLinSpeed = 2

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        self.sub_odom = rospy.Subscriber('/vehicle_blue/odom', Odometry, self.getOdometry)

        self.action_space = spaces.Box(low=np.array([0, -self.maxAngSpeed]),
                                       high=np.array([self.maxLinSpeed, self.maxAngSpeed]), dtype=np.float16)
        self.observation_space = spaces.Box(low=-1, high=100, shape=(4111,), dtype=np.float16)

    def getOdometry(self, odom):

        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)
        return self.position, self.yaw



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
        self.scan_range, minScan = self.scanRange(scan)

        self.rewardMapOld = 96
        state, self.done = self.setStateAndDone(gmap, scan)

        return state

    def step(self, action):

        self.unpause_proxy()

        robgoaly = action[1] - self.position.y
        robgoalx = action[0] - self.position.x
        goal_angle = math.atan2(robgoalx, robgoaly)

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'vehicle_blue/odom'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = action[0] + self.position[0]
        goal.target_pose.pose.position.y = action[1] + self.position[1]
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1

        client.send_goal(goal)
        wait = client.wait_for_result()
        if wait == "rejected":
            os.system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}")
            self.canselled = True

        print(wait)

        gmap = rospy.wait_for_message("/map", OccupancyGrid, timeout=1000)

        self.pause_proxy()

        state, self.done = self.setStateAndDone(gmap, scan)
        reward = self.setReward(self.done, gmap)
        return [state, reward, self.done, {}]

    def setReward(self, done, gmap):

        self.rewardMap = np.sum(np.array(gmap.data) > -1, axis=0)
        self.reward = self.rewardMap - self.rewardMapOld
        self.rewardMapOld = self.rewardMap

        #self.reward += self.rewardObstacleProximity()
        if self.canselled:
            self.reward += -100
            self.canselled = False
        if done:
            self.reward += -100
        return self.reward

    def render(self, mode='human'):
        pass

    def setStateAndDone(self, gmap, scan):

        # # Set state and done

        done = False
        self.scan_range, minScan = self.scanRange(scan)
        if minScan < self.collisionParam or self.TimeoutCounter == 100:
            done = True

        self.splitZones(gmap)
        state = self.zoneValue + self.scan_range + [self.position.x, self.position.y, self.yaw]

        return state, done



    def rewardObstacleProximity(self):
        closestObstacle = min(self.scan_range)
        if closestObstacle <= self.safetyLimit:
            return self.obsProximityParam * -(1 - (closestObstacle / self.safetyLimit))
        else:
            return 0
    def scanRange(self, scan):
    #
        scan_range = []

        for x in range(0, 360, self.lidarDiscretization):
            end = x + self.lidarDiscretization
            scan_range.append(min(scan.ranges[x:end]))

        minScan = min(list(filter(lambda a: a != 0, scan_range[:])))
        return scan_range, minScan

    def splitZones(self, gmap):
        self.zoneValue = []
        arr = np.asarray(gmap.data)
        chunks = np.array_split(arr, 4096)

        for x in range(4096):
            self.zoneValue.append(np.sum(a=chunks[x], axis=0, dtype=np.int32))

        np.array(self.zoneValue) > -1

    def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        return [qx, qy, qz, qw]