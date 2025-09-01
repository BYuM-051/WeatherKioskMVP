from picamera2 import Picamera2
# import pigpio
import time
import cv2
import sys
import os
import serial
from typing import Final

Debug : Final = True

# initialize camera
# OV5647 5MP camera module
# 1. 1080p30 video
# 2. 720p60 video
# 3. 480p60/90 video
picam2 = Picamera2();

frameWidth = 640; # 320, 640, 1280, 1920
frameHeight = 480; # 240, 480, 720, 1080
frameRate = 30; # 30, 60, 90
pixelFormat = "YUV420"; # YUV420, RGB888, BGR888, RGB8888, BGR8888

picam2.configure (
    picam2.create_preview_configuration (
        main = {
            "format": pixelFormat, 
            "size": (frameWidth, frameHeight)
        },
        controls ={"FrameRate": frameRate}
    )
);
picam2.start();
if Debug :
    print("Camera started");
time.sleep(2); # camera warm-up

# configure face detector
base = os.path.dirname(os.path.abspath(__file__));
frontalXmlPath = os.path.join(base, "haarcascade_frontalface_default.xml");
if not os.path.exists(frontalXmlPath):
    sys.exit("Frontal xml file not found");
frontalFaceCascade = cv2.CascadeClassifier(frontalXmlPath);

if frontalFaceCascade.empty():
    sys.exit("error loading frontal cascade classifier");

profileXmlPath = os.path.join(base, "haarcascade_profileface.xml");
if not os.path.exists(profileXmlPath):
    sys.exit("Profile xml file not found");
profileFaceCascade = cv2.CascadeClassifier(profileXmlPath);

if profileFaceCascade.empty():
    sys.exit("error loading profile cascade classifier");

# initialize serial (arduino nano)
nano = serial.Serial('/dev/ttyUSB0', 115200, timeout=1);
time.sleep(2); # serial port warm-up

nano.reset_input_buffer();
nano.reset_output_buffer();

PanDirection : Final = 1; # 1 = normal, -1 = inverted
TiltDirection : Final = -1; # 1 = normal, -1 = inverted

PanLimit : Final = (0, 180);
TiltLimit : Final = (0, 180);

def clamp(value, low, high) :
    return high if value > high else low if value < low else value;

def send_servo_angle(pan = None, tilt = None) :
    p = "0" if pan is None else str(int(pan));
    t = "0" if tilt is None else str(int(tilt));
    nano.write(f"P:{p},T:{t}\n".encode());

# TODO : add servo release code

# these code was deprecated due to using arduino nano for servo control
#===============================================================================================
# # initialize servo
# # 6221MG servo
# # 1. 0 degree = 500us
# # 2. 90 degree = 1500us
# # 3. 180 degree = 2500us

# pi = pigpio.pi();
# if not pi.connected:
#     sys.exit("run pigpio daemon first");

# PanServoPin : Final = 17;
# TiltServoPin : Final = 27;
# pi.set_mode(PanServoPin, pigpio.OUTPUT);
# pi.set_mode(TiltServoPin, pigpio.OUTPUT);

# # function to convert angle to pulse width
# def angle_to_pulse(angle, min_us=500, max_us=2500):
#     angle = max(0.0, min(180.0, float(angle)));
#     return int(min_us + (max_us - min_us) * (angle / 180.0))

# # set servo angle
# def set_servo_angle(servo_pin, angle):
#     pulse_width = angle_to_pulse(angle);
#     pi.set_servo_pulsewidth(servo_pin, pulse_width);

# # servo demonstration code
# # 90 degree (center)
# set_servo_angle(PanServoPin, 90);
# set_servo_angle(TiltServoPin, 90);
# time.sleep(0.5);
# # 0 degree (left/up)
# set_servo_angle(PanServoPin, 0);
# time.sleep(0.5);
# # 180 degree (right/down)
# set_servo_angle(PanServoPin, 180);
# time.sleep(0.5);
# # 90 degree (center)
# set_servo_angle(PanServoPin, 90);
# time.sleep(0.5);

# currentPan = 90;
# currentTilt = 90;
# set_servo_angle(PanServoPin, currentPan);
# set_servo_angle(TiltServoPin, currentTilt);

#===============================================================================================

#servo demontration code with arduino nano
send_servo_angle(90, 90); # center
time.sleep(0.5);
send_servo_angle(0, 90);  # left
time.sleep(0.5);
send_servo_angle(180, 90); # right
time.sleep(0.5);
currentPan = 90;
currentTilt = 90;
send_servo_angle(currentPan, currentTilt);

deadBandWidth : Final = frameWidth / 6; # 1/6 of frame width
deadBandHeight : Final = frameHeight / 6; # 1/6 of frame

lastUpdateTime = int(time.monotonic() * 1000);
lastReturnedTime = int(time.monotonic() * 1000);
searchFlag = 0; # 0 = noSearch, 1 = step1, 2 = step2, 3 = step3
returnPeriod : Final = 2;
searchPeriod : Final = 10;


try : 
    while True:
        try :
            faceFound = False;
            frame = picam2.capture_array();
            gray = frame[:frameHeight, :frameWidth];

            everyFaces = [];
            faces = frontalFaceCascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60));

            if len(faces) > 0:
                faceFound = True;

                if Debug :
                    print(f"frontal faces detected: {len(faces)}");
                (x, y, w, h) = max(faces, key=lambda rect: rect[2] * rect[3]);
                everyFaces.append((x, y, w, h));

            faces = profileFaceCascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60));
            
            if len(faces) > 0:
                faceFound = True;

                if Debug :
                    print(f"right faces detected: {len(faces)}");
                (x, y, w, h) = max(faces, key=lambda rect: rect[2] * rect[3]);
                everyFaces.append((x, y, w, h));

            flippedCapture = cv2.flip(gray, 1);
            faces = profileFaceCascade.detectMultiScale(flippedCapture, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60));
        
            if len(faces) > 0:
                faceFound = True;

                if Debug :
                    print(f"left faces detected: {len(faces)}");
                (x, y, w, h) = max(faces, key=lambda rect: rect[2] * rect[3]);
                everyFaces.append((frameWidth - (x + w), y, w, h));

            if faceFound :
                (x, y, w, h) = max(everyFaces, key=lambda rect: rect[2] * rect[3]);
                face_center_x = x + w // 2;
                face_center_y = y + h // 2;

                # MVP of tracking
                # if X axis is in left side, turn right
                if face_center_x < (frameWidth / 2 - deadBandWidth) :
                    currentPan = clamp(currentPan + 2.0, (PanLimit[0]), (PanLimit[1]));
                elif face_center_x > (frameWidth / 2 + deadBandWidth) :
                    currentPan = clamp(currentPan - 2.0, (PanLimit[0]), (PanLimit[1]));
                # if Y axis is in upper side, turn up
                if face_center_y < (frameHeight / 2 - deadBandHeight) :
                    currentTilt = clamp(currentTilt + 2.0, (TiltLimit[0]), (TiltLimit[1]));
                elif face_center_y > (frameHeight / 2 + deadBandHeight) :
                    currentTilt = clamp(currentTilt - 2.0, (TiltLimit[0]), (TiltLimit[1]));
                
                send_servo_angle(currentPan, currentTilt);
                lastUpdateTime = int(time.monotonic() * 1000);

                #TODO : kill search thread if running

                if Debug :
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2);
                    cv2.circle(frame, (face_center_x, face_center_y), 5, (255, 0, 0), -1);
                    print(f"face center: ({face_center_x}, {face_center_y})");

            else :

                if(searchFlag == 0 and (int(time.monotonic() * 1000) - lastUpdateTime) > (returnPeriod * 1000)) :
                    if currentPan != 90 :
                        currentPan = 90;
                    if currentTilt != 90 :
                        currentTilt = 90;
                    send_servo_angle(currentPan, currentTilt);
                    lastUpdateTime = int(time.monotonic() * 1000);
                    lastReturnedTime = int(time.monotonic() * 1000);
                    # TODO : start search thread if not running
                    # searchFlag = 1;
                
                # TODO : seperate search thread
                # if(searchFlag != 0 and ((int(time.monotonic() * 1000) - lastReturnedTime) > (searchPeriod * 1000))) :
                #     #step 1 : pan left
                #     currentPan = 0;
                #     send_servo_angle(currentPan, currentTilt);
                #     searchFlag = 2;
                #     #step 2 : pan right
                #     currentPan = 180;
                #     send_servo_angle(currentPan, currentTilt);
                #     searchFlag = 3;
                #     #step 3 : center
                #     currentPan = 90;
                #     send_servo_angle(currentPan, currentTilt);
                #     searchFlag = 0;

                # these code was deprecated due to using arduino nano for servo control
                #===============================================================================================
                # if (int(time.monotonic() * 1000) - lastUpdateTime) > (returnPeriod):
                #     if currentPan != 90:
                #         currentPan = 90;
                #         set_servo_angle(PanServoPin, currentPan);
                #     if currentTilt != 90:
                #         currentTilt = 90;
                #         set_servo_angle(TiltServoPin, currentTilt);
                #   lastUpdateTime = int(time.monotonic() * 1000);
                #===============================================================================================

                if Debug :
                    print("no face detected");

            if Debug :
                cv2.imshow("Frame", frame);
                if cv2.waitKey(1) & 0xFF == ord('q') :
                    break;

            time.sleep(0.1);

        except Exception as e:
            print(f"error: {e}");
            time.sleep(0.1);

except KeyboardInterrupt :
    print("teminated by user");
finally :
    print("terminate");
    nano.flush();
    nano.close();
    picam2.stop();
    sys.exit(0);