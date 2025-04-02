import RPi.GPIO as GPIO
import time

# Pin Definitions
M1_IN1 = 17  # Motor 1 Direction Pin 1
M1_IN2 = 27  # Motor 1 Direction Pin 2
M1_ENA = 12  # Motor 1 PWM Speed Control

M2_IN1 = 22  # Motor 2 Direction Pin 1
M2_IN2 = 23  # Motor 2 Direction Pin 2
M2_ENA = 13  # Motor 2 PWM Speed Control

M3_IN1 = 24  # Motor 3 Direction Pin 1
M3_IN2 = 25  # Motor 3 Direction Pin 2
M3_ENA = 21  # Motor 3 PWM Speed Control

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup([M1_IN1, M1_IN2, M1_ENA, M2_IN1, M2_IN2, M2_ENA, M3_IN1, M3_IN2, M3_ENA], GPIO.OUT)

# Initialize PWM on ENA at 1000 Hz
pwm1 = GPIO.PWM(M1_ENA, 1000)
pwm2 = GPIO.PWM(M2_ENA, 1000)
pwm3 = GPIO.PWM(M3_ENA, 1000)

pwm1.start(0)
pwm2.start(0)
pwm3.start(0)

def motor_control(in1, in2, pwm, speed, direction="forward"):
    """Controls a motor's direction and speed."""
    if direction == "forward":
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif direction == "backward":
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    elif direction == "stop":
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(0)
        return
    
    pwm.ChangeDutyCycle(speed)

try:
    while True:  # Infinite loop
        print("Moving all motors forward")
        motor_control(M1_IN1, M1_IN2, pwm1, 100, "forward")
        motor_control(M2_IN1, M2_IN2, pwm2, 100, "forward")
        motor_control(M3_IN1, M3_IN2, pwm3, 100, "forward")
        time.sleep(3)



except KeyboardInterrupt:
    print("\nStopping all motors and exiting...")

finally:
    motor_control(M1_IN1, M1_IN2, pwm1, 0, "stop")
    motor_control(M2_IN1, M2_IN2, pwm2, 0, "stop")
    motor_control(M3_IN1, M3_IN2, pwm3, 0, "stop")
    
    pwm1.stop()
    pwm2.stop()
    pwm3.stop()
    
    GPIO.cleanup()
