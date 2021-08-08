#!/usr/bin/env python3

import cv2
import numpy as np
import depthai as dai
import time
import serial
import socket
import pickle
import struct
import os


ser = serial.Serial('/dev/ttyS1', 115200, timeout=.1)

time.sleep(5)

ser.write(b'p')

def init_device(fps):
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRgb = pipeline.createColorCamera()

    rgbOut = pipeline.createXLinkOut()

    rgbOut.setStreamName("rgb")

    #Properties
    camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setFps(fps)
    # For now, RGB needs fixed focus to properly align with depth.
    # This value was used during calibration
    camRgb.initialControl.setManualFocus(130)

    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 15]

    # Linking
    camRgb.isp.link(rgbOut.input)
    return pipeline

def init_socket():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('192.168.1.8', 8484))
    return client_socket

def draw_frame(frameRgb):    
    frameRgb = cv2.rectangle(frameRgb, (0,100), (240,350), (255,255,255), 2)
    frameRgb = cv2.rectangle(frameRgb, (240,100), (480,350), (255,255,255), 2)
    frameRgb = cv2.rectangle(frameRgb, (480,100), (720,350), (255,255,255), 2)
    return frameRgb 

def divide_frame(frameRgb):
    left = frameRgb[100:350, 0:240]
    mid = frameRgb[100:350, 240:480]
    right = frameRgb[100:350, 480:]
    return left, mid, right

def detect_kp(poi_left, poi_mid, poi_right, orb):
    kp_left, _ = orb.detectAndCompute(poi_left, None)
    kp_mid, _ = orb.detectAndCompute(poi_mid, None)
    kp_right, _ = orb.detectAndCompute(poi_right, None)
    return kp_left, kp_mid, kp_right 

def drawKP(poi_left, kp_left, poi_mid, kp_mid, poi_right, kp_right):
    poi_left = cv2.drawKeypoints(poi_left, kp_left, None)
    poi_mid = cv2.drawKeypoints(poi_mid, kp_mid, None)
    poi_right = cv2.drawKeypoints(poi_right, kp_right, None)
    return poi_left, poi_mid, poi_right

def draw_num_KP(frameRgb, kp_left, kp_mid, kp_right, font):
    cv2.putText(frameRgb,'%s'%(len(kp_left)),(5,470), font, 1, (255, 0, 127), 4)
    cv2.putText(frameRgb,'%s'%(len(kp_mid)),(245,470), font, 1, (255, 0, 127), 4)
    cv2.putText(frameRgb,'%s'%(len(kp_right)),(485,470), font, 1, (255, 0, 127), 4)

def fuse_orb_org(poi_left, poi_mid, poi_right, frameRgb):
    frameRgb[100:350, 0:240] = poi_left
    frameRgb[100:350, 240:480] = poi_mid
    frameRgb[100:350, 480:] = poi_right
    return frameRgb

def send_to_client(client_socket, frame, encoder):
    result, frame = cv2.imencode('.jpg', frame, encoder)
    data = pickle.dumps(frame, 0)
    size = len(data)
    client_socket.sendall(struct.pack(">L", size) + data)
    print('Sending Data')

def forward():
    ser.write(b'w')

def right():
    ser.write(b'd')

def left():
    ser.write(b'a')

def reverse():
    ser.write(b's')

def stop():
    ser.write(b'p')

def obstacle():
    fps = 15
    width = 720
    height = 480
    num_feats = 2000
    thresh = 20
    font = cv2.FONT_HERSHEY_SIMPLEX

    #Initialize device
    pipeline = init_device(fps)
    
    #initialize ORB
    orb = cv2.ORB_create(nfeatures=num_feats)

    #Initialize Socket and encoder
    client_socket = init_socket()
    encoder = [int(cv2.IMWRITE_JPEG_QUALITY), 15]

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        frameRgb = None

        while True:
            latestPacket = {}
            latestPacket["rgb"] = None

            packets = device.getOutputQueue("rgb").tryGetAll()
            if len(packets) > 0:
                latestPacket["rgb"] = packets[-1]

            if latestPacket["rgb"] is not None:
                frameRgb = latestPacket["rgb"].getCvFrame()
                frameRgb = cv2.resize(frameRgb, (width, height))

                #Draw ROI_Frame
                # frameRgb = draw_frame(frameRgb)
                
                #Divide Frame
                poi_left, poi_mid, poi_right = divide_frame(frameRgb)

                #Detect Keypoints (kp)
                kp_left, kp_mid, kp_right = detect_kp(poi_left, poi_mid, poi_right, orb)
                
                #Draw Keypoints (kp)
                # poi_left, poi_mid, poi_right = drawKP(poi_left, kp_left, poi_mid, kp_mid, poi_right, kp_right)
                
                #Put Number of KP on Screen
                # draw_num_KP(frameRgb, kp_left, kp_mid, kp_right, font)

                #Fuse Original frame and ORB Frame
                # frameRgb = fuse_orb_org(poi_left, poi_mid, poi_right, frameRgb)

                if len(kp_left) < thresh or len(kp_mid) < thresh or len(kp_right) < thresh:

                    if len(kp_left) <= 10 and len(kp_mid) <=10 and len(kp_right) <= 10:
                        # cv2.putText(frameRgb,'Stop',(360,20), font, 1, (255, 0, 0), 4)
                        stop()
                        reverse()
                        reverse()
                        reverse()
                        reverse()

                    if len(kp_left) < thresh and len(kp_mid) < thresh and len(kp_right) >= thresh:
                        # cv2.putText(frameRgb,'Right',(360,20), font, 1, (255, 255, 0), 4)
                        right()
                        right()

                    if len(kp_left) < thresh and len(kp_mid) >=thresh and len(kp_right) < thresh:
                        # cv2.putText(frameRgb,'Right',(360,20), font, 1, (255, 255, 0), 4)
                        forward()

                    if len(kp_left) < thresh and len(kp_mid) >=thresh and len(kp_right) >= thresh:
                        # cv2.putText(frameRgb,'Right',(360,20), font, 1, (255, 255, 0), 4)
                        right()
                        right()

                    if len(kp_left) >=thresh and len(kp_mid) < thresh and len(kp_right) < thresh:
                        # cv2.putText(frameRgb,'Left',(360,20), font, 1, (255, 255, 0), 4)
                        left()
                        left()

                    if len(kp_left) >= thresh and len(kp_mid) < thresh and len(kp_right) >= thresh:
                        # cv2.putText(frameRgb,'Left',(360,20), font, 1, (255, 255, 0), 4)
                        left()
                        left()
                        left()

                    if len(kp_left) >= thresh and len(kp_mid) >= thresh and len(kp_right) < thresh:
                        # cv2.putText(frameRgb,'Left',(360,20), font, 1, (255, 255, 0), 4)
                        left()
                        left()

                if len(kp_left) >= thresh and len(kp_mid) >= thresh and len(kp_right) >= thresh:
                    forward()
                    # cv2.putText(frameRgb,'Forward',(360,20), font, 1, (0, 255, 0), 4)

                print('Left\t\tMid\t\tRight')
                print('%d\t\t%d\t\t%d\n'%(len(kp_left), len(kp_mid), len(kp_right)))

                send_to_client(client_socket, frameRgb, encoder)

            if cv2.waitKey(1) == ord('q'):
               break



obstacle()