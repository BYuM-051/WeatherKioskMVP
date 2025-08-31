from picamera2 import Picamera2
import pigpio
import time
import cv2
import sys
import os
from typing import Final

Debug : Final = True

pi = pigpio.pi();
if not pi.connected:
    sys.exit("run pigpio daemon first");

# initialize camera
# OV5647 5MP camera module
# 1. 1080p30 video
# 2. 720p60 video
# 3. 480p60/90 video
picam2 = Picamera2();

frameWidth = 640; # 320, 640, 1280, 1920
frameHeight = 480; # 240, 480, 720, 1080
frameRate = 30; # 30, 60, 90
format = "YUV420"; # YUV420, RGB888, BGR888, RGB8888, BGR8888

picam2.configure (
    picam2.create_preview_configuration (
        main = {
            "format": format, 
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
xml_path = os.path.join(base, "haarcascade_frontalface_default.xml");
if not os.path.exists(xml_path):
    sys.exit("xml file not found");
faceCascade = cv2.CascadeClassifier(xml_path);

if faceCascade.empty():
    sys.exit("error loading cascade classifier");

# initialize servo
# 6221MG servo
# 1. 0 degree = 500us
# 2. 90 degree = 1500us
# 3. 180 degree = 2500us

PanServoPin : Final = 17;
TiltServoPin : Final = 27;
pi.set_mode(PanServoPin, pigpio.OUTPUT);
pi.set_mode(TiltServoPin, pigpio.OUTPUT);

# function to convert angle to pulse width
def angle_to_pulse(angle, min_us=500, max_us=2500):
    angle = max(0.0, min(180.0, float(angle)));
    return int(min_us + (max_us - min_us) * (angle / 180.0))

# set servo angle
def set_servo_angle(servo_pin, angle):
    pulse_width = angle_to_pulse(angle);
    pi.set_servo_pulsewidth(servo_pin, pulse_width);

# servo demonstration code
# 90 degree (center)
set_servo_angle(PanServoPin, 90);
set_servo_angle(TiltServoPin, 90);
time.sleep(0.5);
# 0 degree (left/up)
set_servo_angle(PanServoPin, 0);
time.sleep(0.5);
# 180 degree (right/down)
set_servo_angle(PanServoPin, 180);
time.sleep(0.5);
# 90 degree (center)
set_servo_angle(PanServoPin, 90);
time.sleep(0.5);

currentPan = 90;
currentTilt = 90;
set_servo_angle(PanServoPin, currentPan);
set_servo_angle(TiltServoPin, currentTilt);

deadBand_Angle : Final = 2.0;

lastUpdateTime = int(time.monotonic() * 1000);
returnPeriod : Final = 2;
searchPeriod : Final = 10;


try : 
    while True:
        try :
            frame = picam2.capture_array();
            gray = frame[:frameHeight, :frameWidth];

            # detect faces
            faces = faceCascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30));

            # if faces are detected, track the first face
            if len(faces) > 0:
                update = False;

                if Debug :
                    print(f"faces detected: {len(faces)}");
                (x, y, w, h) = faces[0];
                face_center_x = x + w // 2;
                face_center_y = y + h // 2;

                
                errorX = face_center_x - frameWidth // 2;
                errorY = face_center_y - frameHeight // 2;

                Targetpan = max(0.0, min(180.0, currentPan + 1 * (errorX // 10)));
                Targettilt = max(0.0, min(180.0, currentTilt - 1 * (errorY // 10)));

                if abs(currentPan - Targetpan) > deadBand_Angle :
                    currentPan = Targetpan;
                    update = True;
                if abs(currentTilt - Targettilt) > deadBand_Angle :
                    currentTilt = Targettilt;
                    update = True;
                if update :
                    set_servo_angle(PanServoPin, currentPan);
                    set_servo_angle(TiltServoPin, currentTilt);
                    update = False;
                    lastUpdateTime = int(time.monotonic() * 1000);
                if Debug :
                    print(f"face center: ({face_center_x}, {face_center_y}), pan: {currentPan}, tilt: {currentTilt}");
            else:
                if (int(time.monotonic() * 1000) - lastUpdateTime) > (returnPeriod):
                    if currentPan != 90:
                        currentPan = 90;
                        set_servo_angle(PanServoPin, currentPan);
                    if currentTilt != 90:
                        currentTilt = 90;
                        set_servo_angle(TiltServoPin, currentTilt);
                if Debug :
                    print("no face detected");
            time.sleep(0.1);

        except Exception as e:
            print(f"error: {e}");
            time.sleep(0.1);

except KeyboardInterrupt :
    print("teminated by user");
finally :
    print("terminate");
    # stop servo
    pi.set_servo_pulsewidth(PanServoPin, 0);
    pi.set_servo_pulsewidth(TiltServoPin, 0);
    pi.stop();
    picam2.stop();
    sys.exit(0);