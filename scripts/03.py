#! /usr/bin/env python

import rospy
from utils import arm



rospy.init_node('demo_03', anonymous=False)

left_arm=arm('left')



left_arm.sendPose([0.5,-.25,0.4],[1,0,0,0] )
left_arm.wait_for_pose()

left_arm.sendGrip([0,0,0])
left_arm.wait_for_grip()


#view pose [0.5,-.25,0.4],[1,0,0,0] 
#target z=-0.195252865553
















