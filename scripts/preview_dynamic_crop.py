#!/usr/bin/env python3

"""
This example shows usage of preview image in crop mode with the possibility to move the crop.
Use 'ASD' in order to do it. Images overlap by 1/3. The AI engine wants 1280 x 224 images.
"""

import cv2
import depthai as dai
import numpy as np

# Create pipeline
pipeline = dai.Pipeline()
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setPreviewSize(1280,524)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)

# Define sources and outputs

manip = pipeline.create(dai.node.ImageManip)
camRgb.preview.link(manip.inputImage)
configIn = pipeline.create(dai.node.XLinkIn)
xout = pipeline.create(dai.node.XLinkOut)

configIn.setStreamName('config')
xout.setStreamName("out1")
manip.out.link(xout.input)

manip.initialConfig.setCropRect(0, 0.57252, 1, 1)
manip.setMaxOutputFrameSize(1280 * 524 * 3)

# Linking
configIn.out.link(manip.inputConfig)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Queues
    q = device.getOutputQueue(xout.getStreamName(), maxSize=4, blocking=False)
    configQueue = device.getInputQueue(configIn.getStreamName())

    sendCamConfig = False

    while True:
        in1 = q.tryGet()
        
        # Frame is ready to be shown
        if in1 is not None:
            cv2.imshow("Image", in1.getCvFrame())

        # Update screen
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        elif key in [ord('a'), ord('s'), ord('d')]:
            if key == ord('a'):
                y1 = 0
                y2 = 0.42748
                
            elif key == ord('d'):
                y1 = 0.57252
                y2 = 1
                
            elif key == ord('s'):
                y1 = 0.28626
                y2 = 0.71374
                
            sendCamConfig = True
        
        # Send new config to camera
        if sendCamConfig:
            cfg = dai.ImageManipConfig()
            cfg.setCropRect(0, y1, 1, y2)
            configQueue.send(cfg)
            sendCamConfig = False
