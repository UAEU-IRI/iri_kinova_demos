import numpy as np
import cv2



cameraMatrix=np.array([[1033.519835, 0.000000, 372.760979],
                       [0.000000, 1034.708447, 245.490466],
                       [0.000000, 0.000000, 1.000000]])
distCo=np.array([0.082399, -0.152927, 0.004990, 0.014425, 0.000000])

cap = cv2.VideoCapture(0)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

while(True):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray,dictionary)

    if len(res[0]) > 0:
        cv2.aruco.drawDetectedMarkers(frame,res[0],res[1])
        for i in range(0,len(res[1])):
            if res[1][i]==[74]:
                pose=cv2.aruco.estimatePoseSingleMarkers(res[0][i],0.05,cameraMatrix,distCo)
                cv2.aruco.drawAxis(frame, cameraMatrix, distCo, pose[0], pose[1], 0.02)
    cv2.aruco.drawAxis(frame, cameraMatrix, distCo, np.array([0.,0.,0.]), np.array([0.,0.,0.]), 0.02)
    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

