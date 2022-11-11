#!/usr/bin/python3

import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2.aruco as aruco
from smbus import SMBus
import numpy as np
import pickle
import board
import busio
import time
import cv2

# Global Variables
command = 0x04
desiredPos = 0
lcd_columns = 16
lcd_rows = 2

i2c = busio.I2C(board.SCL, board.SDA)
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.color = [100, 0, 0]

# initialize i2c
addr = 0x08 # bus address
bus = SMBus(1) # indicates /dev/i2c-1

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()

width, height = (640, 480)
camera.resolution = (width, height)
camera.framerate = 20
rawCapture = PiRGBArray(camera, size=camera.resolution)


# Allow the camera to warmup
time.sleep(0.1)

# Set ISO to desired value
camera.iso = 100

# Wait for automatic gain control to settle
time.sleep(2)

# Now fix values
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g

# Camera Parameters
objects = []
with (open("camera_params.p", "rb")) as openfile:
    while True:
        try:
            objects.append(pickle.load(openfile))
        except EOFError:
            break

K = objects[0]['M']
distCoeffs = objects[0]['coefs_dist']

# Create dictionary and parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()
MARKER_LENGTH = 5  # cm

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array

    #key = cv2.waitKey(1) & 0xFF

    # clear the stream in pre
    rawCapture.truncate(0)

    # ---------- PROCESS ----------

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect Markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # Get Pose
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, markerLength=MARKER_LENGTH, cameraMatrix=K, distCoeffs=distCoeffs)

    if ids is None:
#        try:
#            lcd.message = "No ArUco markers         "[:16] + "\n" + "detected                "[:16]
#        except:
#            print("LCD communication failed")

        # Send message to Arduino
        try:
            data = [0] * 5
            bus.write_i2c_block_data(addr, 1, data)
        except:

            print("Arduino communication failed")

    else:
        marker_id = ids[0, 0]
        marker_corners = corners[0]
        rvec = rvecs[0]
        tvec = tvecs[0]

        # Display axis on marker
        #image = aruco.drawAxis(image, cameraMatrix=K, distCoeffs=None, rvec=rvec, tvec=tvec, length = 1)

        # Determine angle
        angle = np.arctan2(tvec[0,0], tvec[0,2])
        dist = tvec[0,2]

#        try:
#            lcd.message = "id: {}                 ".format(marker_id)[0:16] + "\n" + "angle: {}       ".format(np.round(np.rad2deg(angle), 2))[0:16]
#        except:
#            print("LCD communication failed")

        print(np.rad2deg(angle))

        try:
            data = [0]*5
            data[0] = 1

            angle = np.rad2deg(angle)
            angle_bytes = abs(int(angle * 100))
            if angle >= 0:
                data[2] = (angle_bytes>>8) & 0xFF
                data[1] = angle_bytes & 0xFF
            else:
                angle_bytes = ~angle_bytes + 1
                data[2] = (angle_bytes>>8) & 0xFF
                data[1] = angle_bytes & 0xFF

            dist_bytes = abs(int(dist * 100))
            if dist >= 0:
                data[4] = (dist_bytes>>8) & 0xFF
                data[3] = dist_bytes & 0xFF
            else:
                dist_bytes = ~dist_bytes + 1
                data[4] = (dist_bytes>>8) & 0xFF
                data[3] = dist_bytes & 0xFF


            bus.write_i2c_block_data(addr, 1, data)

        except:
            print("Arduino communication failed")
    # Request and Send command:
#    try:
#        data = bus.read_i2c_block_data(addr, 0x00, 4)
#        currentPos = data[0]*2**24 + data[1]*2**16 + data[2]*2**8 + data[3]
#
#        if currentPos >= 2**31:
#            currentPos -= 2**32
#
#        lcd.message = "Pos: {}     \nSet: {}    ".format(currentPos, desiredPos)
#    except:
#        print("Lost message")
#        time.sleep(0.1)

#    image = aruco.drawDetectedMarkers(image, corners)

    # Draw quadrants
#    image = cv2.line(image, (width//2, 0), (width//2, height), color=255)
#    image = cv2.line(image, (0, height//2), (width, height//2), color=255)

    #cv2.imshow("xd", image)

    # -------- END PROCESS --------
    #if key == ord('q'):
    #    break

cv2.destroyAllWindows()
