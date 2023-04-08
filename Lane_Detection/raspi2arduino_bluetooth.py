import cv2
import RPi.GPIO as GPIO
import sys
import numpy as np
import time
import servo_motor
import socket

soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

soc_ip = ('192.168.0.2', 4210)
soc.bind(soc.ip)

while True:
    print("success")
    data, ip = soc.recvform(1024)

    