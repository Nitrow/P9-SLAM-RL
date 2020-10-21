import rospy

import gym
from gym import spaces
import numpy as np
from custom_gym.srv import *

stepper = rospy.ServiceProxy('stepper', StepFunction)




stepper(1)
stepper(1)
stepper(1)
stepper(1)
