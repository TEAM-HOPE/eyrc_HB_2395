#include <AccelStepper.h>

AccelStepper frontWheel(1, 38, 42);
AccelStepper leftWheel(1, 25, 27);
AccelStepper rightWheel(1, 29, 23);

String recData;

int stringStart = 0;
int stringEnd = 0;

float velFront, velRight, velLeft;

void setup() {
  Serial.begin(115200);
  
  // Setting MaxSpeed
  frontWheel.setMaxSpeed(100000);
  leftWheel.setMaxSpeed(100000);
  rightWheel.setMaxSpeed(100000);

}

void loop() {
  if (Serial.available()>0) { //Check if any data is available on Serial
    stringStart = 0;
    stringEnd = 0;

     String recData = Serial.readStringUntil('\n');

    // Parse velFront
    stringEnd = recData.indexOf(' ', stringStart);
    velFront = recData.substring(stringStart, stringEnd).toFloat();
    stringStart = stringEnd + 1; // Skip the space


    // Parse velRight
    stringEnd = recData.indexOf(' ', stringStart);
    velRight = recData.substring(stringStart, stringEnd).toFloat();
    stringStart = stringEnd + 1; // Skip the space


    // Parse velLeft
    stringEnd = recData.length();
    velLeft = recData.substring(stringStart, stringEnd).toFloat();


  }

  frontWheel.setSpeed(velFront);
  rightWheel.setSpeed(velRight);
  leftWheel.setSpeed(velLeft);


  frontWheel.runSpeed();
  leftWheel.runSpeed();
  rightWheel.runSpeed();
}
