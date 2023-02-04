import cv2
import numpy as np
import sys
import time
import math

#차선인식 함수(영상을 흑백전환, canny, houghLine 과정을 거쳐 변환)
def DetectLane(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #흑백 전환, cv2.cvtColor(frame, 3색 -> 흑백)

    #canny 작업을 토대로 영상에서의 외각선 검출
    canny = cv2.Canny(gray, 50, 200, None, 3) #canny 작업(외각선 검출) cv2.Canny(frame, 하위 임계값(픽셀이 갖는 최솟값), 상위 임계값(최댓값), 엣지 영상, 소벨 연산자 마스크 크기(Default값이 3))
    height = canny.shape[0]
    rectangle = np.array([[(0, height), (120, 300), (520, 300), (640, height)]]) #ROI 설정(차선만 감지되도록)(빌드 해보고 카메라 각도에 따라 값 다시 설정)
    mask = np.zeros_like(canny) #위 배열을 0으로 채움(0으로 채운 후에 ROI에 맞춰 외각선 처리하기 위함)
    cv2.fillPoly(mask, rectangle, 255) #canny 작업된 이미지에 차선 부분만 외각선 처리
    masked_image = cv2.bitwise_and(canny, mask) #bit연산을 통해 canny와 mask가 모두 값이 있는 곳에 mask image 생성
    mask_canny = cv2.cvtColor(masked_image, cv2.COLOR_GRAY2BGR) #다시 컬러로 전환

    line = cv2.HoughLinesP(masked_image, 1, np.pi / 180, 20, minLineLength=10, maxLineGap=10) #허프변환 (frame, 배열에서 rho값, theta값, 배열에서 직선으로 판단할 임계값, 검출할 선분의 최소 길이, 직선으로 간주할 최대 엣지 점 간격)
    line_Right = np.empty((0,5), int) #오른쪽 차선
    line_Left = np.empty((0,5), int) #왼쪽 차선
    
    #왼쪽, 오른쪽 차선을 인식하여 선 생성
    if line is not None:
        line2 = np.empty((len(line), 5), int)
        for i in range(0, len(line)):
            temp = 0
            l = line[i][0]
            line2[i] = np.append(line[i], np.array((np.arctan2(l[1] - l[3], l[0] - l[2]) * 180) / np.pi))
            if line2[i][1] > line2[i][3]:
                temp = line2[i][0], line2[i][1]
                line2[i][0], line2[i][1] = line2[i][2], line2[i][3]
                line2[i][2] , line2[i][3] = temp
            if line2[i][0] < 320 and (abs(line2[i][4]) < 170 and abs(line2[i][4]) < 95):
                line_Left = np.append(line_Left, line2[i])
            elif line2[i][0] > 320 and (abs(line2[i][4]) < 170 and abs(line2[i][4]) < 95):
                line_Right = np.append(line_Right, line2[i])     
    line_Left = line_Left.reshape(int(len(line_Left) / 5), 5)
    line_Right = line_Right.reshape(int(len(line_Right) / 5), 5)

    #왼쪽 차선 인식한다면 왼쪽 line 생성
    try:
        line_Left = line_Left[line_Left[:,0].argsort()[-1]]
        degree_Left = line_Left[4]
        cv2.line(mask_canny, (line_Left[0], line_Left[1]), (line_Left[2], line_Left[3]), (255,0,0), 10, cv2.LINE_AA)
    except:
        degree_Left = 0

    #오른쪽 차선 인식한다면 오른쪽 line 생성
    try:
        line_Right = line_Right[line_Right[:,0].argsort()[0]]
        degree_Right = line_Right[4]
        cv2.line(mask_canny, (line_Right[0], line_Right[1]), (line_Right[2], line_Right[3]), (255,0,0), 10, cv2.LINE_AA)
    except:
        degree_Right = 0

    mimg = cv2.addWeighted(frame, 1, mask_canny, 1, 0)  #카메라 영상에 가중치 부여하여 선명도 높임
    return mimg, degree_Left, degree_Right  #return 값으로 영상, 좌,우 인식한 차선에 선 생성

cap = cv2.VideoCapture(-1)  #카메라 영상
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        cv2.imshow('ImageWindow', DetectLane(frame)[0])
        frame = cv2.resize(frame, (640,360))
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
    
cv2.destroyAllWindows()

#이제부터 라즈베리파이, 아두이노 블루투스 통신으로 모터제어 -