#! /usr/bin/env python

import rospy
import actionlib
from time import sleep
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

class arm:
    def __init__(self,prefix):
        self.prefix=prefix
        action_address = '/' + prefix + '_driver/pose_action/tool_pose'
        self.client=actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
        self.client.wait_for_server()
        self.goal = kinova_msgs.msg.ArmPoseGoal()
        action_address = '/' + prefix + '_driver/fingers_action/finger_positions'
        self.client2=actionlib.SimpleActionClient(action_address, kinova_msgs.msg.SetFingersPositionAction)
        self.client2.wait_for_server()
        self.goal2 = kinova_msgs.msg.SetFingersPositionGoal()
        
    def sendPose(self,position, orientation):
        self.goal.pose.header = std_msgs.msg.Header(frame_id=(self.prefix + '_link_base'))
        self.goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
        self.goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
        
        rospy.loginfo('sending pose goal to '+self.prefix+'_arm')
        self.client.send_goal(self.goal)
        
    
    def wait_for_pose(self):
        if self.client.wait_for_result(rospy.Duration(200.0)):
            return self.client.get_result()
        else:
            self.client.cancel_all_goals()
            rospy.logerr('action timed-out. '+self.prefix+'_arm')
            return None
        
          
            
    def sendGrip(self,fingers):
        self.goal2.fingers = kinova_msgs.msg.FingerPosition(finger1=fingers[0],finger2=fingers[1],finger3=fingers[2])
        rospy.loginfo('sending finger positions to '+self.prefix+'_arm')
        self.client2.send_goal(self.goal2)
  
    def wait_for_grip(self):
        if self.client2.wait_for_result(rospy.Duration(200.0)):
            return self.client2.get_result()
        else:
            self.client2.cancel_all_goals()
            rospy.logerr('action timed-out. '+self.prefix+'_arm')
            return None
             
  

rospy.init_node('demo_02', anonymous=False)

left_arm=arm('left')
right_arm=arm('right')


left_arm.sendPose([-0.211215913296,-0.267644703388,0.504357457161],[-0.315036833286,-0.647377789021,0.551541507244,0.42125493288])
right_arm.sendPose([-0.211215913296,-0.267644703388,0.504357457161],[-0.315036833286,-0.647377789021,0.551541507244,0.42125493288])
left_arm.wait_for_pose()
right_arm.wait_for_pose()

left_arm.sendGrip([0,0,0])
right_arm.sendGrip([0,0,0])
right_arm.wait_for_grip()
left_arm.wait_for_grip()


left_arm.sendPose([0.523457407951,-0.367709696293,-0.154760196805],[-0.536136269569,0.461213290691,-0.441729366779,0.552010297775])
left_arm.wait_for_pose()
left_arm.sendGrip([4980.0,4824.0,3000])
left_arm.wait_for_grip()
left_arm.sendGrip([4980.0,4824.0,3000])
left_arm.wait_for_grip()
left_arm.sendPose([-0.561575114727,-0.599759101868,0.360933333635],[0.296416968107,0.645706772804,-0.647194862366,-0.276294261217])
right_arm.sendPose([0.628098726273,-0.481364905834,0.411938995123],[0.771660149097,0.153199225664,0.0182327032089,0.617039859295])
left_arm.wait_for_pose()
right_arm.wait_for_pose()
right_arm.sendGrip([5600,0,5600])
right_arm.wait_for_grip()
left_arm.sendGrip([0,0,0])
left_arm.wait_for_grip()
left_arm.sendPose([-0.211215913296,-0.267644703388,0.504357457161],[-0.315036833286,-0.647377789021,0.551541507244,0.42125493288])
right_arm.sendPose([-0.0181629955769,0.699589550495,0.4],[0.303452044725,0.679972469807,0.531700253487,0.403545737267])
right_arm.wait_for_pose()
left_arm.wait_for_pose()
right_arm.sendPose([-0.0181629955769,0.699589550495,0.263590246439],[0.303452044725,0.679972469807,0.531700253487,0.403545737267])
right_arm.wait_for_pose()
right_arm.sendGrip([0,0,0])
right_arm.wait_for_grip()

right_arm.sendPose([-0.0181629955769,0.699589550495,0.4],[0.303452044725,0.679972469807,0.531700253487,0.403545737267])
right_arm.wait_for_pose()

right_arm.sendPose([-0.211215913296,-0.267644703388,0.504357457161],[-0.315036833286,-0.647377789021,0.551541507244,0.42125493288])
right_arm.wait_for_pose()
























