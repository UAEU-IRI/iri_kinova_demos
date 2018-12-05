import numpy as np
import rospy
import tf
from utils import Arm
from time import sleep
from std_msgs.msg import String


home1=[[0.033291015774,-0.241356492043,0.49616086483],
       [-0.0375369824469,0.997712373734,0.0093100303784,-0.0554469488561]]

home2=[[-0.143723472953,0.370040029287,0.378145724535],
      [0.995715022087,0.069659717381,0.0589518249035,0.0149603504688]]

def callBack(msg):
    id=msg.data
    global listener,gripPerc
    try:
        (trans,rot) = listener.lookupTransform('j2s6s300_link_base',
                                                 "marker"+id, rospy.Time(0))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "_"
        return None

    trans[2]+=0.2
    arm.sendPose(trans,home1[1])
    arm.wait_for_pose()
    trans[2]-=0.13
    arm.sendPose(trans,home1[1])
    arm.wait_for_pose()
    arm.sendGrip([gripPerc,gripPerc,gripPerc])
    arm.wait_for_grip()
    trans[2]+=0.13
    arm.sendPose(trans,home1[1])
    arm.sendPose(home1[0],
      home1[1])
    arm.wait_for_pose()
    arm.sendGrip([0,0,0])
    arm.wait_for_grip()
    
    
    
    
rospy.init_node('pick_node')
arm=Arm('j2s6s300')
rate = rospy.Rate(100)
listener = tf.TransformListener()
arm.sendPose(home1[0],
      home1[1])
arm.wait_for_pose()
gripPerc=4000
arm.sendGrip([0,0,0])
arm.wait_for_grip()

rospy.Subscriber('id', String, callback=callBack,)


cond=False
while not rospy.is_shutdown():
    rate.sleep()
    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

