import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler 

def movebase_client():

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "vehicle_blue/base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1*math.cos(45)
    goal.target_pose.pose.position.y = 1*math.sin(45)
    
    quat = quaternion_from_euler(0, 0, 45)
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")





"""
def add_element(dict, key, value):
    if key not in dict:
        dict[key] = []
    dict[key].append(value)


d = {}
t = ()
s = -60
dist = 3

for i in range(9):
    t = (dist*math.cos(s), dist*math.sin(s), s)
    add_element(d, i, t)
    s += 15

for key, values in d.items():
    print("Action: {},  Goal Position: {}".format(key, values))
"""

