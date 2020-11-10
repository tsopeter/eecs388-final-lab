#!/usr/bin/env python
from __future__ import division

# Imports
import tensorflow as tf
model = __import__("model")
import cv2
import sys
import os
import time
import math
import numpy as np
import picamera
import picamera.array
import serial

# Radian <-> Degree conversion functions
def deg2rad(deg):
        return deg * math.pi / 180.0
def rad2deg(rad):
        return 180.0 * rad / math.pi

# Get and set the number of cores to be used by TensorFlow
if(len(sys.argv) > 1):
	NCPU = int(sys.argv[1])
else:
	NCPU = 1
config = tf.compat.v1.ConfigProto(intra_op_parallelism_threads=NCPU, \
                        inter_op_parallelism_threads=NCPU, \
                        allow_soft_placement=True, \
                        device_count = {'CPU': 1})

# Load the model
sess = tf.compat.v1.InteractiveSession(config=config)
saver = tf.compat.v1.train.Saver()
model_load_path = "model/model.ckpt"
saver.restore(sess, model_load_path)

#name of port
portid1 = "/dev/ttyAMA1"
portid2 = "/dev/ttyAMA2"

#baudrate
portbaudrate = 115200

#serial objects
ser1.Serial(port = portid1, baudrate = portbaudrate);

# Main Loop
with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera, (640, 480)) as rawCapture:

        # Init
        # Set framerate to 20 because dnn processes every 50ms
        camera.resolution = (640, 480)
        camera.framerate = 20

        # Allow the camera to warm up
        time.sleep(0.1)

        for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):

            # Get the raw NumPy array representation of the image
            image = frame.array

            # Each frame is of size 640x480x3 (Three arrays the size of the resolution, corresponding to red/green/blue)
            # The DNN model consumes images sized 200x66 with values between 0 and 1
            # Here, we resize the array to the correct resolution and then convert the RGB range from [0,255] to [0,1]
            img = cv2.resize(image, (200, 66))
            img = img / 255

            # Feed the model and return the predicted angle
            rad = model.y.eval(feed_dict={model.x: [img]})[0][0]
            deg = rad2deg(rad)

            # Displays the image
            cv2.imshow('Frame', image)
            key = cv2.waitKey(1) & 0xFF

            # Clear the stream for the next frame
            rawCapture.truncate(0)

            #send the deg to HiFive Uart1
            data_send = bytes(deg)
            ser1.write(data_send)

            # Repeat until user hits 'q'
            if key == ord('q'):
                break
