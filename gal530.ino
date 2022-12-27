/* PLAYER TWO - GAL 
 *  Gal only sends its mapped position to processing. No simulation is done here.
 */

// Variables:
double rawPos = 0;

// Pin declares
int sensorPosPin = A2; // input pin for MR sensor

void setup() {
  Serial.begin(115200);
  pinMode(sensorPosPin, INPUT); 
  delay(100);
}

void loop() {
  
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor
}
