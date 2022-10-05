#!/usr/bin/python3

import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2.aruco as aruco
from smbus import SMBus
import numpy as np
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
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [100, 0, 0]

# initialize i2c
addr = 0x08 # bus address
bus = SMBus(1) # indicates /dev/i2c-1

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
#camera.rotation = 180
#camera.hflip = True

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


# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array

    key = cv2.waitKey(1) & 0xFF

    # clear the stream in pre
    rawCapture.truncate(0)

    # ---------- PROCESS ----------

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Create dictionary and parameters
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()

    # Detect Markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is None:
            pass
            #print("No markers found")
    else:
        #print(corners[0][0])
        average_point = (int(np.mean(corners[0][0][:,0])), int(np.mean(corners[0][0][:,1])))
        gray = cv2.circle(gray, average_point, radius=5, color=255, thickness=-1)

        centroid = ( np.array(average_point) - np.array([width, height])/2 ) * np.array([1, -1])
        # print(centroid)

        x, y = centroid

        if x>0 and y>0:
            #print("Cuadrante 1")
            command = 0x01
            desiredPos = 800
            # bus.write_byte(addr, 0x01)

        elif x<0 and y>0:
            #print("Cuadrante 2")
            command = 0x02
            desiredPos = 1600
            # bus.write_byte(addr, 0x02)

        elif x<0 and y<0:
            #print("Cuadrante 3")
            command = 0x03
            desiredPos = 2400
            # bus.write_byte(addr, 0x03)

        else:
            #print("Cuadrante 4")
            command = 0x04
            desiredPos = 0
            # bus.write_byte(addr, 0x04)

    # Request and Send command:
    try:
        data = bus.read_i2c_block_data(addr, command, 4)
        currentPos = data[0]*2**24 + data[1]*2**16 + data[2]*2**8 + data[3]

        if currentPos >= 2**31:
            currentPos -= 2**32

        lcd.message = "Pos: {}     \nSet: {}    ".format(currentPos, desiredPos)
    except:
        print("Lost message")
        time.sleep(0.1)

    gray = aruco.drawDetectedMarkers(gray, corners)

    # Draw quadrants
    gray = cv2.line(gray, (width//2, 0), (width//2, height), color=255)
    gray = cv2.line(gray, (0, height//2), (width, height//2), color=255)

    cv2.imshow("", gray)
    
    # -------- END PROCESS --------
    
    
    if key == ord('q'):
        break
    
cv2.destroyAllWindows()

