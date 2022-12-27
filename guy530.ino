/* PLAYER ONE - GUY */
// Pin declares
int ledPin = 4;
int pwmPin = 5; // PWM output pin for motor 1
int pwmPinGal = 6; // PWM output pin for motor 2
int dirPin = 8; // direction output pin for motor 1
int dirPinGal = 7; // direction output pin for motor 2 GAL
int sensorPosPin = A2; // input pin for MR sensor
int sensorPosPinGal = A5; // input pin for gal MR sensor
int fsrPin = A3; // input pin for FSR sensor
int vibeGuy = 2;
int vibeGal = 3; 

// Score Keeping
int guyScore = 0;
int galScore = 0;

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int rawPosGal = 0;               // current raw reading from MR sensor GAL
int lastRawPosGal = 0;        // last raw reading from MR sensor GAL
int lastLastRawPosGal = 0; // last last raw reading from MR sensor GAL
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;

int updatedPosGal = 0;
int flipNumberGal = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffsetGal = 0;
int rawDiffGal = 0;
int lastRawDiffGal = 0;
int rawOffsetGal = 0;
int lastRawOffsetGal = 0;
const int flipThreshGal = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flippedGal = false;

// Kinematics variables
double xh = 0;           // position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;
double mappedGuyPositionInBounds; 
double mappedGalPositionInBounds; 

// Kinematics variables GAL
double xhGal = 0;           // position of the handle [m]
double theta_sGal = 0;      // Angle of the sector pulley in deg
double xh_prevGal;          // Distance of the handle at previous time step
double xh_prev2Gal;
double dxhGal;              // Velocity of the handle
double dxh_prevGal;
double dxh_prev2Gal;
double dxh_filtGal;         // Filtered velocity of the handle
double dxh_filt_prevGal;
double dxh_filt_prev2Gal;

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor
double rh = .08;         
double rp = .0047625;       
double rs = 0.07;        

// Lateral collision force variables
double mu = 10;  //changed from 0.2
double k_puck = 2;
double k_wall = .05; 
double puck_radius = 10;
double delta_y = 0;

// Force output variables GAL
double forceGal = 0;           // force at the handle
double TpGal = 0;              // torque of the motor pulley
double dutyGal = 0;            // duty cylce (between 0 and 255)
unsigned int outputGal = 0;    // output command to the motor
double rhGal = .08;
double rpGal = .0047625;
double rsGal = .07;

// Initialize puck starter speed
double speedVal = 0.5;

struct Puck {
  double x;
  double y;
  double xVel;
  double yVel;
};

struct Human {
  double x;   //location of the leftmost end of the paddle, relative to left wall
  double y;   //height of the paddle, relative to the bottom of the wall
  double width; //length of the paddle
  String condition; //TODO: can used later when we have power ups, for instance, ie a "slow" condition that does things
  double ghostX; // keep track of "ghost" position to calculate wall force
};

double wallWidth = 480;
double wallHeight = 800;
double goalWidth = 200;
double goalLeftBoundary = wallWidth/2.0 - goalWidth/2.0;
double goalRightBoundary = wallWidth/2.0 + goalWidth/2.0;
double malletThickness = 10;

// Constants for time tracking
double counterGal = -1;
double counterGuy = -1;
unsigned long startMillis = 0;
unsigned long currentMillis = 0;
const unsigned long timeBuzz = 3000; // 3000 ms = 3 s

//create puck(s) here
Puck a = {300, 25, speedVal, speedVal};
int input;

//create human players here
Human guy = {wallWidth/2.0, 50, 100, "normal", wallWidth/2.0};
Human gal = {wallWidth/2.0, wallHeight-50, 100, "normal", wallWidth/2.0};

void setup() {
  Serial.begin(115200);
  delay(500);

  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  setPwmFrequency(pwmPinGal,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  pinMode(pwmPinGal, OUTPUT);  // PWM pin for motor A
  pinMode(dirPinGal, OUTPUT);  // dir pin for motor A
  pinMode(vibeGuy, OUTPUT); // Guy vibe motor
  pinMode(vibeGal, OUTPUT); // Gal vibe motor
  analogWrite(vibeGuy, 0);
  analogWrite(vibeGal, 0);
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);

  // Initialize motor Gal
  analogWrite(pwmPinGal, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPinGal, LOW);  // set direction
  
  // Initialize position variables Gal
  lastLastRawPosGal = analogRead(sensorPosPinGal);
  lastRawPosGal = analogRead(sensorPosPinGal);
  
}

void loop() {
  force = 0;
  forceGal = 0;
  digitalWrite(ledPin, HIGH);
  currentMillis = millis();         // get current "time" (number of ms since program started)
  if ((currentMillis - startMillis) > timeBuzz) {
      buzzOff(vibeGuy);
      buzzOff(vibeGal);
    }
  //double inputGalPosition = input.toDouble(); //the input position is in terms of the display screen
  //Serial.println(input, 5);
  // ORIGINAL LINE: double mappedGuyPosition = mapp(xh, -0.03300, 0.03584, 0, wallWidth); //Second and Third arguments vary by hapkit minXh and maxXh calibration. //map(xh, -0.03300, 0.03584, 0, wallWidth)
  double mappedGuyPosition = mapp(xh, -0.02, 0.025, 0, wallWidth); //Second and Third arguments vary by hapkit minXh and maxXh calibration. //map(xh, -0.03300, 0.03584, 0, wallWidth)
  // if the player is at the wall, don't let the mallet go any further:
  if ( mappedGuyPosition - 25 - 15 < 0 ) {
      mappedGuyPositionInBounds = 0 + 25 + 15;
  }
  else if ( mappedGuyPosition + 25 + 15 > 480 ) {
    mappedGuyPositionInBounds = 480 - 25 - 15;
  } else {
      mappedGuyPositionInBounds = mappedGuyPosition;
  }
  setHumanX(guy, mappedGuyPositionInBounds, mappedGuyPosition);
  
  double mappedGalPosition = mapp(xhGal, -0.02, 0.025, 0, wallWidth);
  // if the player is at the wall, don't let the mallet go any further:
  if ( mappedGalPosition - 25 - 15 < 0 ) {
    mappedGalPositionInBounds = 0 + 25 + 15;
  }
  else if ( mappedGalPosition + 25 + 15 > 480 ) {
    mappedGalPositionInBounds = 480 - 25 - 15;
  } else {
    mappedGalPositionInBounds = mappedGalPosition;
  }
  setHumanX(gal, mappedGalPositionInBounds, mappedGalPosition); 
 
  
  detectGoal(a);
  detectPuckHandleCollision(a, guy, gal);
  detectPuckWallCollision(a);
  detectHapticWallCollision(guy);
  detectHapticWallCollisionGal(gal);

  a.x += a.xVel;
  a.y += a.yVel;

  Tp = (force * rh * rp) / rs;
  TpGal = (forceGal * rh * rp) / rs;
  
  Serial.println((String) mappedGuyPositionInBounds + " " + mappedGalPositionInBounds + " " + a.x + " " + a.y + " " + guyScore + " " + galScore);
 
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor
  rawPosGal = analogRead(sensorPosPinGal); // get gal position from second HK MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);

  // Calculate differences between subsequent MR sensor readings GAL
  rawDiffGal = rawPosGal - lastRawPosGal;          //difference btwn current raw position and last raw position
  lastRawDiffGal = rawPosGal - lastLastRawPosGal;  //difference btwn current raw position and last last raw position
  rawOffsetGal = abs(rawDiffGal);
  lastRawOffsetGal = abs(lastRawDiffGal);

  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Update position record-keeping variables GAL
  lastLastRawPosGal = lastRawPosGal;
  lastRawPosGal = rawPosGal;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    if(rawOffset > flipThresh) { // check to see if the data was good and the most current offset is above the threshold
      updatedPos = rawPos + flipNumber*rawOffset; // update the pos value to account for flips over 180deg using the most current offset 
      tempOffset = rawOffset;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPos = rawPos + flipNumber*lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffset = lastRawOffset;
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPos = rawPos + flipNumber*tempOffset; // need to update pos based on what most recent offset is 
    flipped = false;
  }

  // Keep track of flips over 180 degrees GAL
  if((lastRawOffsetGal > flipThreshGal) && (!flippedGal)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiffGal > 0) {        // check to see which direction the drive wheel was turning
      flipNumberGal--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumberGal++;              // ccw rotation
    }
    if(rawOffsetGal > flipThreshGal) { // check to see if the data was good and the most current offset is above the threshold
      updatedPosGal = rawPosGal + flipNumberGal*rawOffsetGal; // update the pos value to account for flips over 180deg using the most current offset 
      tempOffsetGal = rawOffsetGal;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPosGal = rawPosGal + flipNumberGal*lastRawOffsetGal;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffsetGal = lastRawOffsetGal;
    }
    flippedGal = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPosGal = rawPosGal + flipNumberGal*tempOffsetGal; // need to update pos based on what most recent offset is 
    flippedGal = false;
  }


  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************
  double rh = 0.09;   //[m]
  double m = -154; // Calculated using best fit line
  double b = 600;  // Calculated using best fit line 
  double ts = ((double) updatedPos - b) / m;
  xh = rh * ts * (3.14149 / 180); 

  double rp = 0.01905;   //[m] pulley, obtained from component specs
  double rs = 0.085;     //[m] sector
  double damping;            // maximum damping coefficient you can use that feels good
  double J = (rh * rp) / rs;

  // Calculate velocity with loop time estimation
  dxh = (double)(xh - xh_prev) / 0.001;

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9*dxh + 0.1*dxh_prev; 
    
  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;
  
  dxh_prev2 = dxh_prev;
  dxh_prev = dxh;
  
  dxh_filt_prev2 = dxh_filt_prev;
  dxh_filt_prev = dxh_filt;

  //Serial.println(xh,5);

   //*************************************************************
  //*** Section 2b. Compute position in meters GAL *******************
  //*************************************************************

  double rhGal = 0.09;   //[m]
  double mGal = -154; // Calculated using best fit line
  double bGal = 600;  // Calculated using best fit line 
  double tsGal = ((double) updatedPosGal - bGal) / mGal;
  xhGal = rhGal * tsGal * (3.14149 / 180); 
  double rpGal = 0.01905;   //[m] pulley, obtained from component specs
  double rsGal = 0.085;     //[m] sector
  double dampingGal;            // maximum damping coefficient you can use that feels good
  double JGal = (rhGal * rpGal) / rsGal;

  // Calculate velocity with loop time estimation
  dxhGal = (double)(xhGal - xh_prevGal) / 0.001;

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filtGal = .9*dxhGal + 0.1*dxh_prevGal; 
    
  // Record the position and velocity
  xh_prev2Gal = xh_prevGal;
  xh_prevGal = xhGal;
  
  dxh_prev2Gal = dxh_prevGal;
  dxh_prevGal = dxhGal;
  
  dxh_filt_prev2Gal = dxh_filt_prevGal;
  dxh_filt_prevGal = dxh_filtGal;

  //Serial.println(xh,5);



  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal



//*************************************************************
  //*** Section 4b. Force output (do not change) GAL *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(forceGal > 0) { 
    digitalWrite(dirPinGal, HIGH);
  } else {
    digitalWrite(dirPinGal, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  dutyGal = sqrt(abs(TpGal)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (dutyGal > 1) {            
    dutyGal = 1;
  } else if (dutyGal < 0) { 
    dutyGal = 0;
  }  
  outputGal = (int)(dutyGal* 255);   // convert duty cycle to output signal
  analogWrite(pwmPinGal,outputGal);  // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

//given the x and y coordinates of the puck, change the x and y velocities if the puck is outside the width or height of the wall
void detectPuckWallCollision(Puck& p) { //& is necessary because we are passing values by reference
  // Side Walls:
  if ((p.y >= 0) || (p.y <= 800)) {
    if ((p.x <= 0) || (p.x >= wallWidth)) {
    setXVel(p, -p.xVel);
    }
  }
  // Short Walls:
  if ((p.y >= 0 && p.y <= 5) || (p.y <= 800 && p.y >= 795)) {
    if ((p.x <= goalLeftBoundary || p.x >= goalRightBoundary)){
      setYVel(p, -p.yVel);
    }
  }
}


//given the x and y coordinates of the puck, change the x and y velocities if the puck is in contact with human 
void detectPuckHandleCollision(Puck& p, Human& guy, Human& gal) {
  if ((p.x >= guy.x - guy.width/2.0) && (p.x <= guy.x+guy.width/2.0) && (p.y == guy.y + 10.0)) { //malletThickness = 10
    setYVel(p, -p.yVel);
    startMillis = millis();
    buzzOn(vibeGuy); 
    collisionForce(p, "guy");
  }
  else if ((p.x >= gal.x - gal.width/2.0) && (p.x <= gal.x+gal.width/2.0) && (p.y == gal.y - 10.0)) { //malletThickness = 10
    setYVel(p, -p.yVel);
    startMillis = millis();
    buzzOn(vibeGal); 
    collisionForce(p, "gal");
  }
}

//during a collision between puck and handle, exert proportional lateral force
void collisionForce(Puck& p, String person) {
  if (person == "guy") {
    delta_y = (guy.y+(malletThickness/2)) - p.y - puck_radius;
    // Force direction depends on which way the puck is coming from:
    if (p.xVel > 0) {
      force += mu * k_puck * delta_y;
    } else {
      force -= mu * k_puck * delta_y;
    }
  } else if (person == "gal") {
    delta_y = (gal.y-(malletThickness/2)) - p.y - puck_radius;
    if (p.xVel > 0) {
      forceGal += mu * k_puck * delta_y;
    } else {
      forceGal -= mu * k_puck * delta_y;
    }
  }
}

//given the x and y coordinates of the puck, exert a force if the puck is in contact with a wall 
void detectHapticWallCollision(Human& person) {
  if ( (person.ghostX+25) >= 480 ) { // used to be 480, not 340
    force += -k_wall*(480 - (person.ghostX+25)) ; // used to be 480, not 340
  } else if  ( (person.ghostX-25) <= 0) { // used to be 0, not 140
    force += -k_wall*(0 - (person.ghostX-25)); // used to be 0, not 140
  } else {
    force += 0;
  }
}

//given the x and y coordinates of the puck, exert a force if the puck is in contact with a wall 
void detectHapticWallCollisionGal(Human& person) {
  if ( (person.ghostX+25) > 480 ) { // used to be 480, not 340
    forceGal += -k_wall*(480 - (person.ghostX+25)) ; // used to be 480, not 340
  } else if  ( (person.ghostX-25) < 0) { // used to be 0, not 140
    forceGal += -k_wall*(0 - (person.ghostX-25)); // used to be 0, not 140
  } else {
    forceGal += 0;
  }
}


//given the x and y velocities of the puck, check if the puck is in the goal boundary box
void detectGoal(Puck& p) {
  if ((p.y <= 0) && (p.x >= goalLeftBoundary) && (p.x <= goalRightBoundary)) {
    //TODO: implement scoring mechanism? the puck just resets to the center of the screen for now
    setXPos(p, 200);
    setYPos(p, 400);
    long randNumber = random(4);
    //Serial.println((String) "randNumber is: " + randNumber);
    if (randNumber <= 1) {
      setXVel(p, -speedVal);
      setYVel(p, -speedVal);
    }
    else if (randNumber <= 2) {
      setXVel(p, -speedVal);
      setYVel(p, speedVal);
    }
    else if (randNumber <= 3) {
      setXVel(p, speedVal);
      setYVel(p, speedVal);
    }
    else {
      setXVel(p, speedVal);
      setYVel(p, -speedVal);   
    }
    galScore++;
    delay(2000);
  }
  else if ((p.y >= wallHeight) && (p.x >= goalLeftBoundary) && (p.x <= goalRightBoundary)) {
    setXPos(p, 200);
    setYPos(p, 400);
    long randNumber = random(4);
    //Serial.println((String) "randNumber is: " + randNumber);
    if (randNumber <= 1) {
      setXVel(p, -speedVal);
      setYVel(p, -speedVal);
    }
    else if (randNumber <= 2) {
      setXVel(p, -speedVal);
      setYVel(p, speedVal);
    }
    else if (randNumber <= 3) {
      setXVel(p, speedVal);
      setYVel(p, speedVal);
    }
    else {
      setXVel(p, speedVal);
      setYVel(p, -speedVal);   
    }
    guyScore++;
    delay(2000);
  }
}

void buzzOn(int pin){
  digitalWrite(pin, HIGH);  // 30 < 20% * 255(MAX) // turn on vibration motor 
}

void buzzOff(int pin){
  digitalWrite(pin, LOW);  // 30 < 20% * 255(MAX)
}

//given a puck p and a double n, set the x velocity to be the given value
void setXVel(Puck& p, double n){
  p.xVel = n;
}

//given a puck p and a double n, set the y velocity to be the given value
void setYVel(Puck& p, double n){
  p.yVel = n;
}

//given a puck p and a double n, set the x position to be the given value
void setXPos(Puck& p, double n){
  p.x = n;
}

//given a puck p and a double n, set the y position to be the given value
void setYPos(Puck& p, double n){
  p.y = n;
}

//increase the x position of the human. this function is called when the "l" (for player 1) or the "d" key (for player 2) is pressed.
void increaseHumanX(Human& h) {
  h.x += 1;
}

//decrease the x position of the human. this function is called when the "j" (for player 1) or the "a" key (for player 2) is pressed.
void decreaseHumanX(Human& h) {
  h.x -= 1;
}

//given a human h and a double n, set the x position to be the given value
void setHumanX(Human& h, double n, double ghost) {
  h.x = n;
  h.ghostX = ghost;
}

double mapp(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
} 
