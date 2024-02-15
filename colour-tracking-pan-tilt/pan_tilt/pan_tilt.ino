#include <Servo.h>

Servo panservo, tiltservo;

void setup() {
  Serial.begin(9600);
  panservo.attach(9, 544, 2500);
  tiltservo.attach(10, 544, 2500);
}

void loop() {
  if (Serial.available() >= 16) {
    char buffer[8];

    // Read and print received bytes for debugging
    Serial.readBytes(buffer, 8);
    double pan_angle = strtod(buffer, NULL);

    // Read and print received bytes for debugging
    Serial.readBytes(buffer, 8);
    double tilt_angle = strtod(buffer, NULL);

    

    pan_angle = map(pan_angle, -90, 90, 150, 50);
    tilt_angle = map(tilt_angle, -90, 90, 50, 150);

    panservo.write(pan_angle);
    tiltservo.write(tilt_angle);

    Serial.print("Pan Angle: ");
    Serial.println(pan_angle);

    Serial.print("Tilt Angle: ");
    Serial.println(tilt_angle);


    delay(10);  // Add a small delay to stabilize servo movements
  }
}
