import RPi.GPIO as GPIO
import time
import sys

#라즈베리파이to아두이노 블루투스 통신이 안됐을 때
#라즈베리파이에 직접 핀 꽂아서 코딩할 때 사용
GPIO.setwarnings(False) #코드 실행 시 오류가 뜨는 것 방지
GPIO.setmode(GPIO.BOARD) #빵판 사용하면 수정

HIGH = 1
LOW = 0

#RC카 키트에 따라 수정해야 될 듯
#모터보드 어떻게 할지 생각
#서보모터가 연결된 핀
ENA = 11
ENB = 13
DCpin1 = 15 
DCpin2 = 12
DCpin3 = 16
DCpin4 = 18

#서보모터 출력으로 설정
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(DCpin1, GPIO.OUT)
GPIO.setup(DCpin2, GPIO.OUT)
GPIO.setup(DCpin3, GPIO.OUT)
GPIO.setup(DCpin4, GPIO.OUT)

#서보모터 속도 제어
pwm1 = GPIO.PWM(ENA, 100)
pwm2 = GPIO.PWM(ENB, 100)

pwm1.start(0)
pwm2.start(0)

#DCpin1, DCpin2 -> right motor
#DCpin3, DCpin4 -> left motor
#if DCpin1 HIGH -> 2right motor forward
#if DCpin2 HIGH -> 2right motor back
#if DCpin3 HIGH -> 2left motor back
#if DCpin4 HIGH -> 2left motor forward

def Forward(speed):
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    GPIO.output(DCpin1, HIGH)   
    GPIO.output(DCpin2, LOW)
    GPIO.output(DCpin3, LOW)
    GPIO.output(DCpin4, HIGH)

def Back(speed):
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    GPIO.output(DCpin1, LOW)
    GPIO.output(DCpin2, HIGH)
    GPIO.output(DCpin3, HIGH)
    GPIO.output(DCpin4, LOW)

def Stop():
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)
    GPIO.output(DCpin1, LOW)
    GPIO.output(DCpin2, LOW)
    GPIO.output(DCpin3, LOW)
    GPIO.output(DCpin4, LOW)

#좌회전, 우회전 코딩 해야됨
def TurnLeft():
    pwm1.ChangeDutyCycle(30)
    pwm2.ChangeDutyCycle(20)
    GPIO.output(DCpin1, HIGH)   
    GPIO.output(DCpin2, LOW)
    GPIO.output(DCpin3, LOW)
    GPIO.output(DCpin4, HIGH)
    
def TurnRight():
    pwm1.ChangeDutyCycle(20)
    pwm2.ChangeDutyCycle(30)
    GPIO.output(DCpin1, HIGH)   
    GPIO.output(DCpin2, LOW)
    GPIO.output(DCpin3, LOW)
    GPIO.output(DCpin4, HIGH)

def TurnHardLeft():
    pwm1.ChangeDutyCycle(20)
    pwm2.ChangeDutyCycle(35)
    GPIO.output(DCpin1, HIGH)   
    GPIO.output(DCpin2, LOW)
    GPIO.output(DCpin3, LOW)
    GPIO.output(DCpin4, HIGH)

def TurnHardRight():
    pwm1.ChangeDutyCycle(35)
    pwm2.ChangeDutyCycle(20)
    GPIO.output(DCpin1, HIGH)   
    GPIO.output(DCpin2, LOW)
    GPIO.output(DCpin3, LOW)
    GPIO.output(DCpin4, HIGH)


# if __name__ == '__main__':
#     try:
#         while True:
#             Back(12)

#     except KeyboardInterrupt:
#         print("Measurment stopped by user")
#         pwm1.stop()
#         pwm2.stop()
#         GPIO.cleanup()
#         sys.exit()



while(1):
    TurnLeft()

Stop()

pwm1.stop()
pwm2.stop()
GPIO.cleanup()