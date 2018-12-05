import numpy as np
import cv2
import rospy
from utils import Marker
from time import sleep
import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('marker_locator')
ids=[74,113]
markerLength=0.05
arucoDictionary='ORIGINAL' 
RATE=100
vid_input=0
filter=0.9
cameraMatrix=np.array([[1033.519835, 0.000000, 372.760979],
                       [0.000000, 1034.708447, 245.490466],
                       [0.000000, 0.000000, 1.000000]])
distCo=np.array([0.082399, -0.152927, 0.004990, 0.014425, 0.000000])
cap = cv2.VideoCapture(vid_input)
marker={}
for id in ids:
    marker[id]=Marker(id,markerLength,cameraMatrix,distCo,filter=0.8,
                      dictionary='ORIGINAL')

cond=False
rate = rospy.Rate(RATE)
br=tf.TransformBroadcaster()

bridge = CvBridge()
image_pub = rospy.Publisher("markers",Image,queue_size=10)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    for id in ids:
        cond,frame=marker[id].updatePose(gray)
        print marker[id]
        if cond:
            br.sendTransform(marker[id].position[-1],
                             marker[id].rotation,
                             rospy.Time.now(),
                             "marker"+str(id),"camera_link")               
    
    image_pub.publish(bridge.cv2_to_imgmsg(frame))
    rate.sleep()
    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

