import lgpio
import time

# Motor 1
M1_IN1 = 17
M1_IN2 = 27
M1_ENA = 12

# Motor 2
M2_IN1 = 22
M2_IN2 = 23
M2_ENA = 13

# Motor 3
M3_IN1 = 24
M3_IN2 = 25
M3_ENA = 18

# Open GPIO chip
h = lgpio.gpiochip_open(0)

# Set all pins to output
pins = [M1_IN1, M1_IN2, M1_ENA, M2_IN1, M2_IN2, M2_ENA, M3_IN1, M3_IN2, M3_ENA]
for pin in pins:
    lgpio.gpio_claim_output(h, pin, 0)

def motor_forward(in1, in2, ena):
    lgpio.gpio_write(h, in1, 1)
    lgpio.gpio_write(h, in2, 0)
    lgpio.tx_pwm(h, ena, 1000, 100)  # 1kHz, 100% duty

def motor_backward(in1, in2, ena):
    lgpio.gpio_write(h, in1, 0)
    lgpio.gpio_write(h, in2, 1)
    lgpio.tx_pwm(h, ena, 1000, 100)

def motor_stop(in1, in2, ena):
    lgpio.gpio_write(h, in1, 0)
    lgpio.gpio_write(h, in2, 0)
    lgpio.tx_pwm(h, ena, 1000, 100)

try:
    while True:
        print("Motors forward")
        motor_forward(M1_IN1, M1_IN2, M1_ENA)
        motor_forward(M2_IN1, M2_IN2, M2_ENA)
        motor_forward(M3_IN1, M3_IN2, M3_ENA)
        time.sleep(3)

        # print("Motors backward")
        # motor_backward(M1_IN1, M1_IN2, M1_ENA)
        # motor_backward(M2_IN1, M2_IN2, M2_ENA)
        # motor_backward(M3_IN1, M3_IN2, M3_ENA)
        # time.sleep(3)

        # print("Motors stop")
        # motor_stop(M1_IN1, M1_IN2, M1_ENA)
        # motor_stop(M2_IN1, M2_IN2, M2_ENA)
        # motor_stop(M3_IN1, M3_IN2, M3_ENA)
        # time.sleep(2)

except KeyboardInterrupt:
    print("Stopping...")
    motor_stop(M1_IN1, M1_IN2, M1_ENA)
    motor_stop(M2_IN1, M2_IN2, M2_ENA)
    motor_stop(M3_IN1, M3_IN2, M3_ENA)
    lgpio.gpiochip_close(h)
