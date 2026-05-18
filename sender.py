import socket
import cv2
import numpy as np
import struct
import time
import random

MaximumPacketSize = 1400

ip = "192.168.1.139"
broadcast_ip = "192.168.1.255"
port = 8080
''
messageId = 0
packetId = 0

scaleCapture = 1
scaleSend = 3

framePerSecond = 20
packetDelay = 0.0001

camId = 1
print("Connecting camera #0")
camera0 = cv2.VideoCapture(camId)
print("Set #0 width")
camera0.set(cv2.CAP_PROP_FRAME_WIDTH,int(1920/scaleCapture))
print("Set #0 height")
camera0.set(cv2.CAP_PROP_FRAME_HEIGHT,int(1080/scaleCapture))
print("Camera #0 connected")
frame_width0 = int(camera0.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height0 = int(camera0.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera #0 resolution {frame_width0} {frame_height0}")

camId = 2
print("Connecting camera #1")
camera1 = cv2.VideoCapture(camId)
print("Set #1 width")
camera1.set(cv2.CAP_PROP_FRAME_WIDTH,int(1920/scaleCapture))
print("Set #1 height")
camera1.set(cv2.CAP_PROP_FRAME_HEIGHT,int(1080/scaleCapture))
print("Camera #1 connected")
frame_width1 = int(camera1.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height1 = int(camera1.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera #1 resolution {frame_width1} {frame_height1}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind((ip,port))

frame_width = max(frame_width0,frame_width1)
frame_height = max(frame_height0,frame_height1)
print(f"Frame resolution {frame_width} {frame_height}")
frameCount = 0
while True:
    ret, frame0 = camera0.read()
    ret, frame1 = camera1.read()
    
    frame0 = cv2.resize(frame0,(int(frame_width/scaleSend),int(frame_height/scaleSend)))
    frame1 = cv2.resize(frame1,(int(frame_width/scaleSend),int(frame_height/scaleSend)))

    frame = np.vstack((frame0,frame1))
    
    cv2.imshow("to send", frame)
    
    _, encoded = cv2.imencode(".jpg",frame)
    data = encoded.tobytes()
    
    bufferToSend = struct.pack("I",len(data)) + data
    dataLength = len(bufferToSend)
    remainingBytes = dataLength
    currentIndex = 0
    packetCount = 0
    while remainingBytes > 0:
        headerSize = 8
        toSend = remainingBytes + headerSize
        if toSend > MaximumPacketSize:
            toSend = MaximumPacketSize - headerSize
        try:
            byteSent = sock.sendto(struct.pack('I',packetCount) +
                                   struct.pack('I',frameCount) +
                                   bufferToSend[currentIndex:currentIndex+toSend-headerSize],
                                   (broadcast_ip,port))
            if byteSent <= 0 or byteSent != toSend:
                raise ValueError("Error on send while sending frame")
        except socket.error:
            raise ValueError("Error on socket while sending frame")
        time.sleep(packetDelay)
        currentIndex += byteSent-headerSize
        remainingBytes -= byteSent-headerSize
        packetCount += 1

    ch = chr(cv2.waitKey(int(1000/framePerSecond)) & 0xFF)
    if ch=='q' or ch=='Q': break
    frameCount += 1

sock.close()