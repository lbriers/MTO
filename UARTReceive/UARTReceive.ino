#define MOTOR_PIN 9  // PWM pin for motor control

void setup() {
    Serial.begin(115200);  // Set baud rate to match Raspberry Pi
    pinMode(MOTOR_PIN, OUTPUT);
    analogWrite(MOTOR_PIN, 0);  // Start with motor off
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Read command from UART
        int duty_cycle = input.toInt();  // Convert to integer

        if (duty_cycle >= 0 && duty_cycle <= 100) {
            int pwm_value = map(duty_cycle, 0, 100, 0, 255);  // Convert to 8-bit PWM range
            analogWrite(MOTOR_PIN, pwm_value);
            Serial.print("Motor Speed Set: ");
            Serial.print(duty_cycle);
            Serial.println("%");
        } else {
            Serial.println("Invalid duty cycle received.");
        }
    }
}