import cv2
import numpy as np
import depthai as dai
import serial
import time
import random
import os
import socket
import pickle
import struct
import numpy as np

output = os.popen('dmesg | grep tty')
output = output.read()
output = output.split('\n')[-2].split(' ')[-1]

try:
    ser = serial.Serial('/dev/%s'%(output), 115200, timeout=.1)
    print('ESP connected at port: %s '%(output))
except:
    try:
        ser = serial.Serial('/dev/ttyS1', 115200, timeout=.1)
        print('ESP connected to UART: /dev/ttyS1')
    except:
        print('Serial not connected')

time.sleep(2)

ser.write(b'p')

fps = 15
width = 720
height = 480
conf = 255
thresh = 0.1
#ser.write(b'H')
monoResolution = dai.MonoCameraProperties.SensorResolution.THE_400_P

pipeline = dai.Pipeline()
queueNames = []

left = pipeline.createMonoCamera()
right = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()

depthOut = pipeline.createXLinkOut()

depthOut.setStreamName("depth")
queueNames.append("depth")

#Properties

# For now, RGB needs fixed focus to properly align with depth.
# This value was used during calibration


left.setResolution(monoResolution)
left.setBoardSocket(dai.CameraBoardSocket.LEFT)
left.setFps(fps)
right.setResolution(monoResolution)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
right.setFps(fps)

stereo.initialConfig.setConfidenceThreshold(conf)
# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)
#stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

# Linking
left.out.link(stereo.left)
right.out.link(stereo.right)
stereo.disparity.link(depthOut.input)
font = cv2.FONT_HERSHEY_SIMPLEX
# Connect to device and start pipeline

def send_to_client(client_socket, frame, encoder):
    result, frame = cv2.imencode('.jpg', frame, encoder)
    data = pickle.dumps(frame, 0)
    size = len(data)
    client_socket.sendall(struct.pack(">L", size) + data)
    print('Sending Data')

def init_socket():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('192.168.1.8', 8181))
    return client_socket

client_socket = init_socket()
encoder = [int(cv2.IMWRITE_JPEG_QUALITY), 15]

with dai.Device(pipeline) as device:
    device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    frameDepth = None

    while True:
    
        latestPacket = {}
        latestPacket["depth"] = None

        img_tmp = np.zeros((height, width))


        queueEvents = device.getQueueEvents(("depth"))
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]

        if latestPacket["depth"] is not None:
            frameDepth = latestPacket["depth"].getFrame()
            maxDisparity = stereo.getMaxDisparity()
            # Optional, extend range 0..95 -> 0..255, for a better visualisation
            if 1: frameDepth = (frameDepth * 255. / maxDisparity).astype(np.uint8)
            # Optional, apply false colorization
            if 1: frameDepth = cv2.applyColorMap(frameDepth, cv2.COLORMAP_HOT)
            frameDepth = np.ascontiguousarray(frameDepth)
            frameDepth = cv2.resize(frameDepth, (width, height))

            frameCpy = np.copy(frameDepth)
            frameCpy = cv2.rectangle(frameCpy, (0,100), (240,350), (255,255,255), 2)
            frameCpy = cv2.rectangle(frameCpy, (240,100), (480,350), (255,255,255), 2)
            frameCpy = cv2.rectangle(frameCpy, (480,100), (720,350), (255,255,255), 2)

            frameDepth = cv2.cvtColor(frameDepth, cv2.COLOR_BGR2GRAY)

            mask = frameDepth > 235

            img_tmp[mask] = frameDepth[mask]

            img_tmp = cv2.rectangle(img_tmp, (0,100), (240,350), (255,255,255), 2)
            img_tmp = cv2.rectangle(img_tmp, (240,100), (480,350), (255,255,255), 2)
            img_tmp = cv2.rectangle(img_tmp, (480,100), (720,350), (255,255,255), 2)

            poi_left = img_tmp[100:350, 0:240]
            poi_mid = img_tmp[100:350, 240:480]
            poi_right = img_tmp[100:350, 480:]

            num_white_left = np.sum(poi_left >= 240)
            kp_left = num_white_left/(np.shape(poi_left)[0]*np.shape(poi_left)[1])

            num_white_mid = np.sum(poi_mid >= 240)
            kp_mid = num_white_mid/(np.shape(poi_mid)[0]*np.shape(poi_mid)[1])

            num_white_right = np.sum(poi_right >= 240)
            kp_right = num_white_right/(np.shape(poi_right)[0]*np.shape(poi_right)[1])

            cv2.putText(frameDepth,'%0.2f'%(kp_left),(5,470), font, 1, (255, 255, 255), 4)
            cv2.putText(frameDepth,'%0.2f'%(kp_mid),(245,470), font, 1, (255, 255, 255), 4)
            cv2.putText(frameDepth,'%0.2f'%(kp_right),(485,470), font, 1, (255, 255, 255), 4)

            if kp_left > thresh or kp_mid > thresh or kp_right > thresh:

                if kp_left >= 4 and kp_mid >= 4 and kp_right >= 4:
                    cv2.putText(frameDepth,'Stop',(360,20), font, 1, (255, 255, 255), 4)
                    ser.write(b'p')
                    ser.write(b's')
                    ser.write(b's')
                    ser.write(b's')
                    ser.write(b's')

                if kp_left > thresh and kp_mid > thresh and kp_right <= thresh:
                    cv2.putText(frameDepth,'Right',(360,20), font, 1, (255, 255, 255), 4)
                    ser.write(b'd')
                    ser.write(b'd')

                if kp_left > thresh and kp_mid <= thresh and kp_right > thresh:
                    cv2.putText(frameDepth,'Right',(360,20), font, 1, (255, 255, 255), 4)
                    ser.write(b'w')

                if kp_left > thresh and kp_mid <= thresh and kp_right <= thresh:
                    cv2.putText(frameDepth,'Right',(360,20), font, 1, (255, 255, 255), 4)
                    ser.write(b'd')
                    ser.write(b'd')

                if kp_left <= thresh and kp_mid > thresh and kp_right > thresh:
                    cv2.putText(frameDepth,'Left',(360,20), font, 1, (255, 255, 255), 4)
                    ser.write(b'a')
                    ser.write(b'a')

                if kp_left <= thresh and kp_mid > thresh and kp_right <= thresh:
                    cv2.putText(frameDepth,'Left',(360,20), font, 1, (255, 255, 255), 4)
                    ser.write(b'a')
                    ser.write(b'a')
                    ser.write(b'a')

                if kp_left <= thresh and kp_mid <= thresh and kp_right > thresh:
                    cv2.putText(frameDepth,'Left',(360,20), font, 1, (255, 255, 255), 4)
                    ser.write(b'a')
                    ser.write(b'a')

            if kp_left <= thresh and kp_mid <= thresh and kp_right <= thresh:
                ser.write(b'w')
                cv2.putText(frameDepth,'Forward',(360,20), font, 1, (255, 255, 255), 4)

            frameDepth[100:350, 0:240] = poi_left
            frameDepth[100:350:, 240:480] = poi_mid
            frameDepth[100:350:, 480:] = poi_right
            
            frameDepth = cv2.cvtColor(frameDepth, cv2.COLOR_GRAY2BGR)
            frame = np.vstack((frameCpy, frameDepth))
            send_to_client(client_socket, frame, encoder)
            # cv2.imshow('frameDepth', frame)

            frameDepth = None
            frame = None
            frameCpy = None

        if cv2.waitKey(1) == ord('q'):
            ser.write(b'p')
            break
            


