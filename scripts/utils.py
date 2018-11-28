import rospy
import actionlib
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
        action_address = '/' + prefix + '_driver/joints_action/joint_angles'
        self.client3=actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmJointAnglesAction)
        self.client3.wait_for_server()
        self.goal3 = kinova_msgs.msg.ArmJointAnglesGoal()
        
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
            
    def sendAngles(self,angles):
        self.goal3.angles=kinova_msgs.msg.JointAngles(joint1=angles[0],joint2=angles[1],joint3=angles[2],joint4=angles[3],joint5=angles[4],joint6= angles[5], joint7=0.0)
        rospy.loginfo('sending joint angles '+self.prefix+'_arm')
        self.client3.send_goal(self.goal3)

    def wait_for_angles(self):
        if self.client3.wait_for_result(rospy.Duration(200.0)):
            return self.client3.get_result()
        else:
            self.client3.cancel_all_goals()
            rospy.logerr('action timed-out. '+self.prefix+'_arm')
            return None     
             
  
