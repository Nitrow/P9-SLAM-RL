import rospy
import gym
from gym import spaces
import numpy as np
from tf import TransformListener
import os
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, geometry_msgs, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.srv import GetMap
from std_srvs.srv import Empty
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



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
        self.safetyLimit = 2
        self.collisionParam = 0.5
        self.obsProximityParam = 10
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
        self.xDistance = 1
        self.yDistance = 1

        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

        self.action_space = spaces.Box(low=np.array([- self.xDistance,  -self.yDistance]),
                                       high=np.array([self.xDistance , self.yDistance]), dtype=np.float16)
        self.observation_space = spaces.Box(low=-1, high=100, shape=(20,), dtype=np.float16)

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
        linear_vel = 0
        ang_vel = 0

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel
        vel_cmd.angular.z = ang_vel
        self.pub.publish(vel_cmd)


        time.sleep(1)
        gmap = rospy.wait_for_message("/map", OccupancyGrid, timeout=1000)

        self.pause_proxy()
        self.getPose()


        self.rewardMapOld = np.sum(a=splitZones(gmap), axis=0, dtype=np.int64)

        state = splitZones(gmap) + [self.position[0], + self.position[1], self.quaternion[2],
                                                      self.quaternion[3]]
        return state

    def step(self, action):

        self.TimeoutCounter += 1
        self.unpause_proxy()
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
        client.wait_for_result()

        gmap = rospy.wait_for_message("/map", OccupancyGrid, timeout=1000)

        self.pause_proxy()



        state, self.done = self.setStateAndDone(gmap)
        reward = self.setReward(state, self.done)
        return [state, reward, self.done, {}]

    def setReward(self, state, done):
        self.rewardMap = np.sum(a=state, axis=0, dtype=np.int64)
        self.reward = self.rewardMap - self.rewardMapOld
        self.rewardMapOld = self.rewardMap

        #self.reward += -1
        if done:
            self.reward += -1000

        #print(self.reward)
        return self.reward

    def render(self, mode='human'):
        pass

    def setStateAndDone(self, gmap):

        # # Set state and done

        done = False
        self.getPose()

        state = splitZones(gmap) + [self.position[0], + self.position[1], self.quaternion[2],
                                                      self.quaternion[3]]

        return state, done

    def getPose(self):
        if self.tf.frameExists("vehicle_blue/base_link") and self.tf.frameExists("vehicle_blue/odom"):
            t = self.tf.getLatestCommonTime("vehicle_blue/base_link", "vehicle_blue/odom")
            self.position, self.quaternion = self.tf.lookupTransform("vehicle_blue/base_link", "vehicle_blue/odom", t)
        return self.position, self.quaternion

