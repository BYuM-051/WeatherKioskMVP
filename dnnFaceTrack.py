from picamera2 import Picamera2
import pigpio
import time
import cv2
import sys
import os
from typing import Final

Debug : Final = True

# Servo / GPIO init
pi = pigpio.pi()
if not pi.connected:
    sys.exit("run pigpio daemon first")

PanServoPin: Final = 17
TiltServoPin: Final = 27
pi.set_mode(PanServoPin, pigpio.OUTPUT)
pi.set_mode(TiltServoPin, pigpio.OUTPUT)

# JX 6221MG: 500us(0°) ~ 1500us(90°) ~ 2500us(180°)
def angle_to_pulse(angle: float, min_us: int = 500, max_us: int = 2500) -> int:
    a = max(0.0, min(180.0, float(angle)))
    return int(min_us + (max_us - min_us) * (a / 180.0))

def set_servo_angle(pin: int, angle: float):
    pi.set_servo_pulsewidth(pin, angle_to_pulse(angle))

#servo initialize
#Direction: 1 = normal, -1 = inverted
PanDirection  : Final = 1
TiltDirection : Final = -1

currentPan  = 90.0
currentTilt = 90.0
set_servo_angle(PanServoPin, currentPan)
set_servo_angle(TiltServoPin, currentTilt)
time.sleep(0.8)

#servo demonstration
set_servo_angle(PanServoPin, 0.0)   # left
time.sleep(0.3)
set_servo_angle(PanServoPin, 180.0) # right
time.sleep(0.3)
set_servo_angle(PanServoPin, 90.0)  # center

# Camera init (BGR888 MODE)
picam2 = Picamera2()

frameWidth  = 640   # 320, 640, 1280, 1920
frameHeight = 480   # 240, 480, 720, 1080
frameRate   = 30     # 30, 60, 90
format      = "BGR888"  # YUV420, RGB888, BGR888, etc.

picam2.configure(
    picam2.create_preview_configuration(
        main={"format": format, "size": (frameWidth, frameHeight)},
        controls={"FrameRate": frameRate},
    )
)
picam2.start()
if Debug :
    print("Camera started")
time.sleep(2)  # camera warmup

# DNN face detector (Caffe SSD)
base  = os.path.dirname(os.path.abspath(__file__))
proto = os.path.join(base, "deploy.prototxt")
model = os.path.join(base, "res10_300x300_ssd_iter_140000.caffemodel")

if not (os.path.exists(proto) and os.path.exists(model)):
    sys.exit("DNN model files not found (deploy.prototxt / res10_300x300_ssd_iter_140000.caffemodel)")

net = cv2.dnn.readNetFromCaffe(proto, model)

if Debug :
    print("DNN model loaded")

returnPeriodMs : Final = 3000   # 얼굴 미검출시 센터 복귀
detect_thresh: Final  = 0.6     # DNN confidence threshold

lastUpdateMs = int(time.monotonic() * 1000)

def clamp(v, lo, hi):
    return hi if v > hi else lo if v < lo else v

try:
    while True:
        try:
            frame = picam2.capture_array()

            blob = cv2.dnn.blobFromImage(
                frame, scalefactor=1.0, size=(300, 300),
                mean=(104.0, 177.0, 123.0), swapRB=False, crop=False
            )
            net.setInput(blob)
            detection = net.forward()

            faces = []
            h, w = frame.shape[:2]
            N = detection.shape[2]
            for i in range(N):
                conf = float(detection[0, 0, i, 2])
                if conf < detect_thresh:
                    continue
                x1 = int(detection[0, 0, i, 3] * w)
                y1 = int(detection[0, 0, i, 4] * h)
                x2 = int(detection[0, 0, i, 5] * w)
                y2 = int(detection[0, 0, i, 6] * h)

                x1 = clamp(x1, 0, w - 1)
                y1 = clamp(y1, 0, h - 1)
                x2 = clamp(x2, 0, w - 1)
                y2 = clamp(y2, 0, h - 1)
                if x2 <= x1 or y2 <= y1:
                    continue

                faces.append((x1, y1, x2 - x1, y2 - y1))

            if faces:
                (x, y, ww, hh) = max(faces, key=lambda r: r[2] * r[3])
                cx, cy = x + ww // 2, y + hh // 2

                #MVP of tracking
                if cx < frameWidth / 3 :
                    currentPan += 2.0 * PanDirection
                elif cx > frameWidth * 2 / 3 :
                    currentPan -= 2.0 * PanDirection

                if cy < frameHeight / 3 :
                    currentTilt += 2.0 * TiltDirection
                elif cy > frameHeight * 2 / 3 :
                    currentTilt -= 2.0 * TiltDirection

                set_servo_angle(PanServoPin, currentPan)
                set_servo_angle(TiltServoPin, currentTilt)
                lastUpdateMs = int(time.monotonic() * 1000)
                
                if Debug :
                    print(f"face: {len(faces)} | center=({cx},{cy}) | pan={currentPan:.1f} tilt={currentTilt:.1f}")

            else:
                # 얼굴 미검출: returnPeriodMs 지난 뒤 센터로 복귀
                now = int(time.monotonic() * 1000)
                if (now - lastUpdateMs) > returnPeriodMs:
                    changed = False
                    if currentPan != 90.0:
                        currentPan = 90.0
                        set_servo_angle(PanServoPin, currentPan)
                        changed = True
                    if currentTilt != 90.0:
                        currentTilt = 90.0
                        set_servo_angle(TiltServoPin, currentTilt)
                        changed = True
                    if changed:
                        print("return to center")
                        lastUpdateMs = now
                else:
                    print("no face detected")

            time.sleep(0.03)  # ~33ms (약 30fps 루프)

        except Exception as e:
            print(f"error: {e}")
            time.sleep(0.1)

except KeyboardInterrupt:
    print("terminated by user")
finally:
    print("terminate")
    pi.set_servo_pulsewidth(PanServoPin, 0)
    pi.set_servo_pulsewidth(TiltServoPin, 0)
    pi.stop()
    picam2.stop()
    sys.exit(0)