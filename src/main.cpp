#include <Arduino.h>
#include "ScaraRobot.h"
#include "UartRobot.h"



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Robot.begin();
  // HandleRobot.begin(17, 16, 115200);
}



void loop() {
  // put your main code here, to run repeatedly:
  HandleRobot.loop();
}