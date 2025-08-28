import pigpio

pi = pigpio.pi()
pi.set_mode(17, pigpio.OUTPUT)
pi.set_mode(27, pigpio.OUTPUT)
pi.set_servo_pulsewidth(17, 1500) # center position
pi.set_servo_pulsewidth(27, 1500) # center position