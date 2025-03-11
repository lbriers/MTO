#define ENB 6   // Speed control for Motor B
#define IN3 9   // Motor B direction
#define IN4 10  // Motor B direction

void setup() {
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    Serial.begin(9600); // Initialize Serial Monitor

    // Set motor direction (Forward)
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void loop() {
    // Increase speed from 0 to 255
    /*for (int speed = 0; speed <= 255; speed++) {
        analogWrite(ENB, speed);
        Serial.print("Speed: ");
        Serial.println(speed);
        delay(20); // Smooth acceleration
    }
    delay(1000); // Hold max speed for 1 second

    // Decrease speed from 255 to 0
    for (int speed = 255; speed >= 0; speed--) {
        analogWrite(ENB, speed);
        Serial.print("Speed: ");
        Serial.println(speed);
        delay(20); // Smooth deceleration
    }*/
    analogWrite(ENB, 100);

    delay(1000); // Hold at stop for 1 second
}

