
// ME327 Spring 2022
// Final Project: Haptic Air Hockey: Processing Code
// Eva Nates, Ellie Prince, James Wright, Raymond Zhen

// Serial Communication:
import processing.serial.*;
Serial[] myPorts = new Serial[1];  // Create a list of objects from Serial class

// Initialize Other Variables:
float inByte = 0; //current value of the first variable in the string
float lastByte = 0; //previous value of the first variable in the string
float inByte2 = 0; //current value of the second variable in the string
float lastByte2 = 0; //previous value of the second variable in the string
float prev_playerOnePosition = 0;
float prev_playerTwoPosition = 0;
float prev_puckX = 0;
float prev_puckY = 0;
float prev_guyScore = 0;
float prev_galScore = 0;

// Customizable Variables: 
float goalWidth = 200;             // default = 200
float malletWidth = 50;             // default = 30
float playerDistanceFromWall = 50;  // default = 30
float playerOnePosition = width/2;
float playerTwoPosition = width/2;
float puckX; 
float puckY;
float puckDiameter = 20;
float malletThickness = 10;
int guyScore;
int galScore;



void setup () {
  
  // Set Window Size:
  size(480, 800);        

  // Serial Communication (Uncomment Once Ready):
  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].
  //note you may need to change port number, it is 9 for me
  // get the ports' names:
  String portOne = Serial.list()[12]; // for eva laptop: 13 if both in, 12 if just guy

  // open the ports:
  myPorts[0] = new Serial(this, portOne, 115200);  // Player 1
  // A serialEvent() is generated when a newline character is received :
  myPorts[0].bufferUntil('\n');    // Double check this 
}


void draw () {
  
  // Set Background Color:
  //background(0); 
  background(105,105,105);
  
  // Draw Walls:
  stroke(0, 0, 0);
  strokeWeight(20);
  line(0, 0, 0, height);
  line(width, 0, width, height);
  line(0, 0, (width/2)-(goalWidth/2), 0);
  line(width, 0, (width/2)+(goalWidth/2), 0);
  line(0, height, (width/2)-(goalWidth/2), height);
  line(width, height, (width/2)+(goalWidth/2), height);
  
  // Draw Users' Mallets:
  strokeWeight(10);
  stroke(255, 80, 80);
  line(playerOnePosition-(malletWidth/2), playerDistanceFromWall, playerOnePosition+(malletWidth/2), playerDistanceFromWall);
  stroke(102, 153, 255);
  line(playerTwoPosition-(malletWidth/2), height-playerDistanceFromWall, playerTwoPosition+(malletWidth/2), height-playerDistanceFromWall);
  
  // Draw Puck:
  stroke(0, 0, 0);
  strokeWeight(5);
  fill(255, 204, 0);
  circle(puckX, puckY, puckDiameter);  
  
  // Draw Classic Air Hockey Dots:
  strokeWeight(1);
  for (int i = width/11; i < width-20; i = i+(width/11)) {
    for (int j = height/18; j < height-20; j = j+(height/18)) {
      stroke(148, 146, 139);
      circle(i, j, 1);
    }
  }

  // Draw Score:

  // All Black: 
  //fill(0, 0, 0);
  //textSize(30);
  //textAlign(RIGHT);
  //text(guyScore + " - " + galScore, 455, 40);
  
  // Guy Score:
  fill(255, 80, 80); // in same color as Guy Paddle
  textSize(30);
  textAlign(RIGHT);
  text(guyScore, 410, 40);
  
  // Score Dash:
  fill(0, 0, 0);
  textSize(30);
  textAlign(CENTER);
  text(" - ", 420, 40);
  
  // Gal Score:
  fill(102, 153, 255);  // in same color as Gal Paddle
  textSize(30);
  textAlign(LEFT);
  text(galScore, 430, 40);
  
}


void serialEvent (Serial myPort) {

    // Read & Convert Item 1 (Guy Position In-Bounds):
    String s1 = myPort.readStringUntil(' ');
    if (s1==null) {
      return;
    }
    String playerOnePosition_str = trim(s1);
    playerOnePosition = float(playerOnePosition_str);
     
    // Read & Convert Item 2 (Gal Position In-Bounds):
    String s2 = myPort.readStringUntil(' ');
    if (s2==null) {
      return;
    }
    String playerTwoPosition_str = trim(s2);
    playerTwoPosition = float(playerTwoPosition_str);
    
    // Read & Convert Item 3 (Puck X):
    String s3 = myPort.readStringUntil(' ');
    if (s3==null) {
      return;
    }
    String puckX_str = trim(s3);
    puckX = float(puckX_str);
    
    // Read & Convert Item 4 (Puck Y):
    String s4 = myPort.readStringUntil(' ');
    if (s4==null) {
      return;
    }
    String puckY_str = trim(s4);
    puckY = float(puckY_str);
    // If puck is hitting gal:
    //if (puckY + (puckDiameter/2) > (height - playerDistanceFromWall - malletThickness)) {
    //  puckY =  height - playerDistanceFromWall - malletThickness - (puckDiameter/2);
    //}
    
    // Read & Convert Item 5 (Guy Score):
    String s5 = myPort.readStringUntil(' ');
    if (s5==null) {
      return;
    }
    guyScore = int(trim(s5));
    
    // Read & Convert Item 6 (Gal Score):
    String s6 = myPort.readStringUntil('\n');
    if (s6==null) {
      return;
    }
    galScore = int(trim(s6));
    
    
    // If the number is NaN, set current value to previous value:
    if ( Float.isNaN(playerOnePosition) ) {
      playerOnePosition = prev_playerOnePosition;
    //} else {
      //user_on_screen = map(xh, -.05, .05, 0, width); // (can be deleted if no mapping is needed)
    }
    
    if ( Float.isNaN(playerTwoPosition) ) {
      playerTwoPosition = prev_playerTwoPosition;
    //} else {
      //user_on_screen = map(xh, -.05, .05, 0, width); // (can be deleted if no mapping is needed)
    }
    
    // Ensure that player mallets never go through wall:
    //if ( playerOnePosition - 25 - 15 < 0 ) {
    //  playerOnePosition = 0 + 25 + 15;
    //}
    //if ( playerOnePosition + 25 + 15 > 480 ) {
    //  playerOnePosition = 480 - 25 - 15;
    //}
    //if ( playerTwoPosition - 25 - 15 < 0 ) {
    //  playerTwoPosition = 0 + 25 + 15;
    //}
    //if ( playerTwoPosition + 25 + 15 > 480 ) {
    //  playerTwoPosition = 480 - 25 - 15;
    //}
    
    if ( Float.isNaN(puckX) ) {
      puckX = prev_puckX;
    //} else {
      //user_on_screen = map(xh, -.05, .05, 0, width); // (can be deleted if no mapping is needed)
    }
    if ( Float.isNaN(puckY) ) {
      puckY = prev_puckY;
    //} else {
      //user_on_screen = map(xh, -.05, .05, 0, width); // (can be deleted if no mapping is needed)
    }
   
}
