import numpy as np
import cv2
import rospy
import tf




rospy.init_node('markert_tf_broadcaster')


markerID=74
markerLength=0.05
arucoDictionary='ORIGINAL' 
RATE=100
vid_input=0
filter=0.8
rate = rospy.Rate(RATE)
cameraMatrix=np.array([[1033.519835, 0.000000, 372.760979],
                       [0.000000, 1034.708447, 245.490466],
                       [0.000000, 0.000000, 1.000000]])
distCo=np.array([0.082399, -0.152927, 0.004990, 0.014425, 0.000000])


br = tf.TransformBroadcaster()
cap = cv2.VideoCapture(vid_input)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)


avg_P_oc_o=[]

while not rospy.is_shutdown():
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray,dictionary)

    if len(res[0]) > 0:
        cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])
        for i in range(0,len(res[1])):
            count=0
            P_oc_o=[]
            R_co=[]
            if res[1][i]==[markerID]:
                count+=1
                pose=cv2.aruco.estimatePoseSingleMarkers(res[0][i],markerLength,cameraMatrix,distCo)
                rot,jac= cv2.Rodrigues(pose[0])
                P_o_oi=np.matrix([[pose[1][0][0][0]],
                                  [pose[1][0][0][1]],
                                  [pose[1][0][0][2]]])
                R_io=np.matrix(rot)  
                P_i_ic=np.matrix([[0.],[0.],
                                  [-markerLength*0.5]])            
                #computer Position of cube ceter (c) wrt camera frame (o)
                # frame i is the frame attached on cube's face.
                
                #append so if more than one face is detected, we do averging
                #of all cumputed center position and orientation
                P_oc_o.append(P_o_oi+R_io*P_i_ic)
                R_co.append(tf.transformations.quaternion_from_euler(0, 0, 0))
            
            if count>0:
                avg_P_oc_o.append(sum(P_oc_o)/count)
                #on this line, and for the first loop, avg_P_oc_o will
                #have a length of one. Dublicate so we can do the filter
                #equation without changing it
                if len(avg_P_oc_o)<2:
                    print 'here'
                    avg_P_oc_o.append(sum(P_oc_o)/count)  
                #low pass filtering                  
                avg_P_oc_o[-1]=avg_P_oc_o[-1]*(1-filter)+avg_P_oc_o[-2]*filter
                avg_R_co=R_co[0]
                br.sendTransform(avg_P_oc_o[-1],
                                 avg_R_co,
                                 rospy.Time.now(),
                                 "marker"+str(markerID),"camera_link")
                del(avg_P_oc_o[0])                   
                cv2.aruco.drawAxis(frame, cameraMatrix, distCo, pose[0],
                                   pose[1], 0.02)
    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break        
    rate.sleep()
    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

