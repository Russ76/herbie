#!/usr/bin/env python3
"""
This script for Raspberry Pi
The code is edited from docs (https://docs.luxonis.com/projects/api/en/latest/samples/Yolo/tiny_yolo/)
We add parsing from JSON files that contain configuration.
Thanks to Github, Roboflow (image tools), OpenCV, Yolov5 (AI), Google Colab (training), Luxonis
to save all terminal output to a text file, type "script filename.txt" and at the end (after execution), type "exit"
"""
from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import argparse
import json
import blobconverter
import rospy
from std_msgs.msg import Int32
from time import sleep
import RPi.GPIO as GPIO
from gpiozero import Servo

z = ([0,0,0,0,0,0,0,0])
r = ([0,0,0,0,0,0,0,0])
distance = 0

# Python code for Raspberry Pi 4b 4Gig model. 
# Don't use Raspi board pin numbers, but operation numbers
# Use pigpio for better servo control. See its documentation for installation.
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
# start with herbicide pump motor off
GPIO.output(17, GPIO.HIGH)
# Signal line for servo is 25, using 6 volt battery power for quick servo response
# Signal line for relay is 17, using 3.3V line from Raspi to coil
# Power for pump (through relay contacts) comes from 5VDC from TP Link USB hub.
# This USB 3.0 hub also powers camera. Raspi can't do it. Hub powered from 12V battery of yardbot.
# Servo is for aiming sprayer wand. This is smaller wand (folds once), about 18 inches long.
servo = Servo(25)

print()
pmp = input(" Remember to prime pump motor...")

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument("-m", "--model", help="Provide model name or model path for inference",
                    default='~/catkin_ws/src/herbie/supporting/openvino/result_openvino_2021.4_6shave.blob', type=str)
parser.add_argument("-c", "--config", help="Provide config path for inference",
                    default='~/catkin_ws/src/herbie/supporting/openvino/result.json', type=str)
args = parser.parse_args()

# parse config
configPath = Path(args.config)
if not configPath.exists():
    raise ValueError("Path {} does not exist!".format(configPath))

with configPath.open() as f:
    config = json.load(f)
nnConfig = config.get("nn_config", {})

# parse input shape
if "input_size" in nnConfig:
    W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

# extract metadata
metadata = nnConfig.get("NN_specific_metadata", {})
classes = metadata.get("classes", {})
coordinates = metadata.get("coordinates", {})
anchors = metadata.get("anchors", {})
anchorMasks = metadata.get("anchor_masks", {})
iouThreshold = metadata.get("iou_threshold", {})
confidenceThreshold = metadata.get("confidence_threshold", {})

print(metadata)

# parse labels
nnMappings = config.get("mappings", {})
labels = nnMappings.get("labels", {})

# get model path
nnPath = args.model
if not Path(nnPath).exists():
    print("No blob found at {}. Looking into DepthAI model zoo.".format(nnPath))
    nnPath = str(blobconverter.from_zoo(args.model, shaves = 6, zoo_type = "depthai", use_cache=True))
# sync outputs
syncNN = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)

# Properties
# Full width on X, narrow on Y dimension
# Camera is 18 inches above lawn, image shows about 18-20 inches width
# This width chosen because Yardbot was also a Mowbot!
# Preview size is 1280 x 224. This includes good detail
camRgb.setPreviewSize(1280, 524)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
# camRgb.setVideoSize(1920, 320)
camRgb.setFps(16)
# Faster video not needed, I think, at this point.

manip = pipeline.create(dai.node.ImageManip)
camRgb.preview.link(manip.inputImage)
configIn = pipeline.create(dai.node.XLinkIn)
xout = pipeline.create(dai.node.XLinkOut)

configIn.setStreamName('config')
xout.setStreamName("out1")
manip.out.link(xout.input)

# choose from one of three slices of preview image, according to machine speed
manip.initialConfig.setCropRect(0, 0.57252, 1, 1)
manip.setMaxOutputFrameSize(1280 * 524 * 3)

# Linking
configIn.out.link(manip.inputConfig)

# xoutRgb = pipeline.create(dai.node.XLinkOut)
nnOut = pipeline.create(dai.node.XLinkOut)

# xoutRgb.setStreamName("rgb")
nnOut.setStreamName("nn")

# Network specific settings
detectionNetwork.setConfidenceThreshold(confidenceThreshold)
detectionNetwork.setNumClasses(classes)
detectionNetwork.setCoordinateSize(coordinates)
detectionNetwork.setAnchors(anchors)
detectionNetwork.setAnchorMasks(anchorMasks)
detectionNetwork.setIouThreshold(iouThreshold)
detectionNetwork.setBlobPath(nnPath)
detectionNetwork.setNumInferenceThreads(2)
detectionNetwork.input.setBlocking(False)

# Linking
manip.out.link(detectionNetwork.input) # line changed
detectionNetwork.passthrough.link(xout.input) # line changed
detectionNetwork.out.link(nnOut.input)


def callback(msg):
    # rospy.loginfo(rospy.get_caller_id() + "distance_value= %s", msg.data)
    distance = msg.data
    # print ("Distance = ", distance)
    # default is middle slice. These overlap about 1/3
    y1 = 0.28626
    y2 = 0.71374
    if distance > 5:
        y1 = 0
        y2 = 0.42748

    elif distance < 3:
        y1 = 0.57252
        y2 = 1
                
    sendCamConfig = True
    cfg = dai.ImageManipConfig()
    cfg.setCropRect(0, y1, 1, y2)
    configQueue.send(cfg)
    sendCamConfig = False
                
def listener():
    rospy.init_node('listener', anonymous=False)
    sub = rospy.Subscriber("distance_value", Int32, callback)
    

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    t0 = time.time()
    tm = str(t0)
    print('Time = ',tm)
    qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
    qRgb = device.getOutputQueue(name="out1", maxSize=4, blocking=False)
    configQueue = device.getInputQueue(configIn.getStreamName())
    sendCamConfig = False

    frame = None
    detections = []
    startTime = time.monotonic()
    counter = 0
    color2 = (255, 255, 255)
    
    listener()
    

    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def displayFrame(name, frame, detections):
        color = (255, 0, 0)
        global z
        global r
        z = ([0,0,0,0,0,0,0,0]) # x position for directing aim
        r = ([0,0,0,0,0,0,0,0]) # confidence for deciding the aim
        tz = 0
        for detection in detections:
            bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            aim = (detection.xmin + detection.xmax) / 2
            aim = int(aim * 15) - 7
            aim = int(aim) /10
            if aim < -0.7: aim = -0.7
            if aim > 0.7: aim = 0.7
            z[tz] = aim
            # print("Aim =", aim)
            # print("xmin= ",detection.xmin)
            # print("xmax= ",detection.xmax)
            cv2.putText(frame, labels[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            # cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            r[tz] = (detection.confidence)
            tz = tz + 1 # get eight values for confidence, select largest one to decide aim value
            if tz > 7 :
                print(" Slow down! Thick weeds! ")
                break
            # cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        # Show the frame
        # cv2.imshow(name, frame)
        pass


    while True:
      
        inDet = qDet.get()
        in1 = qRgb.get()
        
        if in1 is not None:
            frame = in1.getCvFrame()
            cv2.putText(frame, "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color2)

        if inDet is not None:
            detections = inDet.detections
            counter += 1
            # print("Counter =",counter)
            max_value = None
            max_idx = None
            for idx, num in enumerate(r):
                if (max_value is None or num > max_value):
                    max_value = num
                    max_idx = idx
                    
            confidence = max_value
            aim = z[max_idx] # Pluck proper aim value from list
            # print('Confidence =', max_value, "Aim = ", z[max_idx])
            if confidence > 0.6 :
                servo.value = aim
                print("Servo aim=  ", aim, "  Spraying...  ", "Counter= ", counter)
                # Run pump for proper time. Turn on valve on top of gallon jug!
                GPIO.output(17, GPIO.LOW)
                # print("Spraying now...")
                # target.write("Spraying now...")
                # target.write("\n")
                sleep(0.4)
                # For one half of a second run pump
                GPIO.output(17, GPIO.HIGH)
            else :
                print("No weeds found... ")

        
        if frame is not None:
            displayFrame("out1", frame, detections)
            # print("Only grass... ")
            # pass

        if cv2.waitKey(1) == ord('q'):
            t0 = time.time()
            tm = str(t0)
            print('Time = ',tm)
            sys.exit('See ya later!')
            break
            
