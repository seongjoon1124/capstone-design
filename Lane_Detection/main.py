import cv2
import numpy as np
import sys
import time
import math
import test


cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(3,640)
cap.set(4,360)

while True:
    ret, frame = cap.read()    
    cv2.imshow('video', frame)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
cv2.destroyAllWindows()
