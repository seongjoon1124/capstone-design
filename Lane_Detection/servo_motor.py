# import RPi.GPIO as GPIO
# import time
# import sys

# #라즈베리파이to아두이노 블루투스 통신이 안됐을 때
# #라즈베리파이에 직접 핀 꽂아서 코딩할 때 사용
# GPIO.setwarnings(False) #코드 실행 시 오류가 뜨는 것 방지
# GPIO.setmode(GPIO.BOARD)

# #서보모터가 연결된 핀
# SMpin1 = ""
# SMpin2 = ""
# SMpin3 = ""
# SMpin4 = ""

# #서보모터 출력으로 설정
# GPIO.setup(SMpin1, GPIO.OUT)
# GPIO.setup(SMpin2, GPIO.OUT)
# GPIO.setup(SMpin3, GPIO.OUT)
# GPIO.setup(SMpin4, GPIO.OUT)

# #서보모터 속도 제어
# SMpwm1 = GPIO.PWM(SMpin1, "속도")
# SMpwm2 = GPIO.PWM(SMpin2, "속도")
# SMpwm3 = GPIO.PWM(SMpin3, "속도")
# SMpwm4 = GPIO.PWM(SMpin4, "속도")

# SMpwm1.start(0)
# SMpwm2.start(0)
# SMpwm3.start(0)
# SMpwm4.start(0)

# def Forward(speed):
#     SMpwm1.ChangeDutyCycle(speed)
#     SMpwm2.ChangeDutyCycle(speed)

# def Back(speed):
#     SMpwm3.ChangeDutyCycle(speed)
#     SMpwm4.ChangeDutyCycle(speed)

# def Stop():
#     SMpwm1.ChangeDutyCycle(0)
#     SMpwm2.ChangeDutyCycle(0)
#     SMpwm3.ChangeDutyCycle(0)
#     SMpwm4.ChangeDutyCycle(0)