#! /usr/bin/env python
import rospy
import actionlib
from time import sleep
from kinova_msgs.msg import ArmPoseActionGoal
from kinova_msgs.msg import SetFingersPositionActionGoal

rospy.init_node('demo1', anonymous=False)
pub = rospy.Publisher('/left_driver/pose_action/tool_pose/goal', ArmPoseActionGoal, queue_size=10)
pub2 = rospy.Publisher('/left_driver/fingers_action/finger_positions/goal', SetFingersPositionActionGoal, queue_size=10)
rate=rospy.Rate(1)


g1l = ArmPoseActionGoal()
g1l.header.frame_id= "left_link_base"
g1l.goal.pose.header.frame_id= "left_link_base"
g1l.goal.pose.pose.position.x= 0.523457407951
g1l.goal.pose.pose.position.y= -0.367709696293
g1l.goal.pose.pose.position.z=  -0.154760196805
g1l.goal.pose.pose.orientation.x= -0.536136269569
g1l.goal.pose.pose.orientation.y= 0.461213290691
g1l.goal.pose.pose.orientation.z=  -0.441729366779
g1l.goal.pose.pose.orientation.w=  0.552010297775



grip1l=SetFingersPositionActionGoal()
grip1l.header.frame_id=''
grip1l.goal.fingers.finger1=4980.0
grip1l.goal.fingers.finger2=4824.0
grip1l.goal.fingers.finger3=0.0

grip2l=SetFingersPositionActionGoal()
grip2l.goal.fingers.finger1=4980.0
grip2l.goal.fingers.finger2=4824.0
grip2l.goal.fingers.finger3=0.0







g2l = ArmPoseActionGoal()
g2l.header.frame_id= "left_link_base"
g2l.goal.pose.header.frame_id= "left_link_base"
g2l.goal.pose.pose.position.x= -0.561575114727
g2l.goal.pose.pose.position.y= -0.599759101868
g2l.goal.pose.pose.position.z=  0.360933333635
g2l.goal.pose.pose.orientation.x= 0.296416968107
g2l.goal.pose.pose.orientation.y= 0.645706772804
g2l.goal.pose.pose.orientation.z=  -0.647194862366
g2l.goal.pose.pose.orientation.w=  -0.276294261217

g1l = ArmPoseActionGoal()
g1l.header.frame_id= "left_link_base"
g1l.goal.pose.header.frame_id= "left_link_base"
g1l.goal.pose.pose.position.x= 0.523457407951
g1l.goal.pose.pose.position.y= -0.367709696293
g1l.goal.pose.pose.position.z=  -0.154760196805
g1l.goal.pose.pose.orientation.x= -0.536136269569
g1l.goal.pose.pose.orientation.y= 0.461213290691
g1l.goal.pose.pose.orientation.z=  -0.441729366779
g1l.goal.pose.pose.orientation.w=  0.552010297775


g1l = ArmPoseActionGoal()
g1l.header.frame_id= "left_link_base"
g1l.goal.pose.header.frame_id= "left_link_base"
g1l.goal.pose.pose.position.x= 0.523457407951
g1l.goal.pose.pose.position.y= -0.367709696293
g1l.goal.pose.pose.position.z=  -0.154760196805
g1l.goal.pose.pose.orientation.x= -0.536136269569
g1l.goal.pose.pose.orientation.y= 0.461213290691
g1l.goal.pose.pose.orientation.z=  -0.441729366779
g1l.goal.pose.pose.orientation.w=  0.552010297775

while not rospy.is_shutdown():
    rate.sleep()

    pub.publish(g1l)
    sleep(10)
    pub2.publish(grip1l)
    sleep(2)
    pub.publish(g2l)
    sleep(10)
    pub2.publish(grip2l)
    sleep(2)
    rate.sleep()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
