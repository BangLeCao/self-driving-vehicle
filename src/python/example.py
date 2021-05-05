import numpy as np 
import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node('My_Example')
pub = rospy.Publisher('PCDAT', Float64MultiArray, queue_size=10)
rate = rospy.Rate(0.5)
msg = Float64MultiArray()

while not rospy.is_shutdown():
    msg.data = np.array([0.3, 0.72576699])
    pub.publish(msg)
    rate.sleep()