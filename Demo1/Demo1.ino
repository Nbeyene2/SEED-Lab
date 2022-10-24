#include <DualMC33926MotorShield.h>
DualMC33926MotorShield md;
#include <Wire.h>
#include <Encoder.h>
Encoder myEnc(2, 5); //set encoder pins for motorL
Encoder myEnc1(3, 6); //set encoder pins for motorR

#define msg_size 32 //define size of max recieve message
#define resetn 4 //define reset pin (active low)
#define PWM_outL 9 //define output PWM pin left
#define directionPinL 7 //define pin controlling direction left
#define PWM_outR 10 //define output PWM pin right
#define directionPinR 8 //define pin controlling direction right
uint8_t msg_recieve[msg_size];

void setup() {
  Serial.begin(115200); //set baud rate
  pinMode(PWM_outL, OUTPUT); //assign output PWM pin left
  pinMode(PWM_outR, OUTPUT); //assign output PWM pin right
  
  digitalWrite(resetn, HIGH); //toggle reset pin
  analogWrite(PWM_outL, 0); //initialize PWM left
  analogWrite(PWM_outR, 0); //initialize PWM right
  md.init(); //initialize motor shield
  analogWrite(PWM_outL,0); //initialize PWM left
  analogWrite(PWM_outR,0); //initialize PWM right
  Wire.begin(0x8); //join I2C bus as slave with address 8
  Wire.onReceive(receiveEvent); //call recieveEvent when data received
  Wire.onRequest(requestEvent); //call requestEvent when data requested

}
//Change these variables as input
char direc = 'R';
long angle = 360;
long dist = 12;
//**********************
bool turn = true;
long elapsedTime = 0;
bool change = false;


//Initialize position and motor controller variables for left motor
long currentPositionL = -999; //initialize current Position
long newPositionL = 0; //initialize new Position
long desiredPositionL = 0; //initialize desired Position from Aruco marker
double changeL = (2*3.14159265)/3200; //delta value (2pi/3200)
double currentThetaL = 0; //initialize current theta value
double newThetaL = 0; //initialize the new theta value
double voltageL = 0; //initialize voltage level
double currentVoltageL = 8.0; //set voltage cap
int PWML = 0; //initialize PWM output to motor
double currentTimeL = 0; //initialize current time
double newTimeL = 0; //initialize the new time
double errorL = 0; //initialize error value
double KpL = 8; //initialize Kp
double KiL = 0.0008; //initialize Ki
double currentIntegralL = 0; //initialize current integral term
double newIntegralL = 0; //initialize new integral term
double futureThetaL = 0; //initialize future theta term
double futurePositionL = 0; //initialize future position term
int AngVelL = 0;

//Initialize position and motor controller variables for right motor
long currentPositionR = -999; //initialize current Position
long newPositionR = 0; //initialize new Position
long desiredPositionR = 0; //initialize desired Position from Aruco marker
double changeR = (2*3.14159265)/3200; //delta value (2pi/3200)
double currentThetaR = 0; //initialize current theta value
double newThetaR = 0; //initialize the new theta value
double voltageR = 0; //initialize voltage level
double currentVoltageR = 8.0; //set voltage cap
int PWMR = 0; //initialize PWM output to motor
double currentTimeR = 0; //initialize current time
double newTimeR = 0; //initialize the new time
double errorR = 0; //initialize error value
double KpR = 8; //initialize Kp
double KiR = 0.0008; //initialize Ki
double currentIntegralR = 0; //initialize current integral term
double newIntegralR = 0; //initialize new integral term
double futureThetaR = 0; //initialize future theta term
double futurePositionR = 0; //initialize future position term
int AngVelR = 0;

//TODO ---Add right wheel code to mirror left wheel past this point---

void loop() {
  //controller left variable equations
  currentTimeL = (millis()/1000.0); //find current time in seconds
  currentThetaL = newThetaL; //reset the new theta with the old theta
  newIntegralL = currentIntegralL + (errorL*changeL); //In = Io + (e*d)*I
  currentIntegralL = newIntegralL; //reset old integral term
  //futurePosition = futureTheta * 509; //x = theta * P
  errorL = futurePositionL - newPositionL; //e = x1 - x0
  voltageL = KpL * (errorL * changeL) + (KiL * newIntegralL); //v = Kp * (e*d) + (Ki * In)
  PWML = (voltageL/currentVoltageL)*255; //set PWM output
  if (PWML > 255) { 
    PWML = 255; //if above max value, set at cap
  }
  else if (PWML < -255) {
    PWML = -255; //if below min value, set at floor
  }
  if (PWML >= 0) {
    digitalWrite(directionPinL, LOW); //set direction if positive
  }
  if (PWML < 0) {
    digitalWrite(directionPinL, HIGH); //set direction if negative
  }
  analogWrite(PWM_outL, abs(PWML)); //set PWM pulse absolute value
/*
  //print to serial monitor
  Serial.print("Desired: ");
  Serial.print(futurePosition);
  Serial.print("\tActual: ");
  Serial.print(newPosition);
  Serial.print("\tPWM value: ");
  Serial.print(PWM);
  Serial.print("\tError: ");
  Serial.println(error);
*/
  Serial.println(newPositionL);
  // Speed limit
  newPositionL = myEnc.read(); //read encoder value
  //Serial.println(newPositionL); 
  if (newPositionL != currentPositionL) { //if detected change in position
    if (newPositionL > currentPositionL) { //if increased position
      newThetaL = newThetaL + (changeL * ((float)(newPositionL - currentPositionL))); //correct by subtracting difference
    }
    else { //if decreased position
      newThetaL = newThetaL - (changeL * ((float)(currentPositionL - newPositionL))); //correct by adding difference
    }
    currentPositionL = newPositionL; //reset position
  }
  newTimeL = (millis()/1000.0); //find end time in seconds
  delay(8 - ((newTimeL - currentTimeL)/1000)); //add delay
  //Serial.print(currentPositionL);
  //Serial.print("\t");
  //Serial.println(newPositionL);
  /*
  if (millis() > 1000) {
    futurePosition = 3000;
  }
  AngVel = 2*3.14*newPosition/(30*(millis()/1000));
  Serial.println(AngVel);
  */
  //controller right variable equations
  currentTimeR = (millis()/1000.0); //find current time in seconds
  currentThetaR = newThetaR; //reset the new theta with the old theta
  newIntegralR = currentIntegralR + (errorR*changeR); //In = Io + (e*d)*I
  currentIntegralR = newIntegralR; //reset old integral term
  //futurePosition = futureTheta * 509; //x = theta * P
  errorR = futurePositionR - newPositionR; //e = x1 - x0
  voltageR = KpR * (errorR * changeR) + (KiR * newIntegralR); //v = Kp * (e*d) + (Ki * In)
  PWMR = (voltageR/currentVoltageR)*255; //set PWM output
  if (PWMR > 255) { 
    PWMR = 255; //if above max value, set at cap
  }
  else if (PWMR < -255) {
    PWMR = -255; //if below min value, set at floor
  }
  if (PWMR >= 0) {
    digitalWrite(directionPinR, LOW); //set direction if positive
  }
  if (PWMR < 0) {
    digitalWrite(directionPinR, HIGH); //set direction if negative
  }
  analogWrite(PWM_outR, abs(PWMR)); //set PWM pulse absolute value
/*
  //print to serial monitor
  Serial.print("Desired: ");
  Serial.print(futurePosition);
  Serial.print("\tActual: ");
  Serial.print(newPosition);
  Serial.print("\tPWM value: ");
  Serial.print(PWM);
  Serial.print("\tError: ");
  Serial.println(error);
*/
  Serial.println(newPositionR);
  // Speed limit
  newPositionR = myEnc1.read(); //read encoder value
  //Serial.println(newPositionL); 
  if (newPositionR != currentPositionR) { //if detected change in position
    if (newPositionR > currentPositionR) { //if increased position
      newThetaR = newThetaR + (changeR * ((float)(newPositionR - currentPositionR))); //correct by subtracting difference
    }
    else { //if decreased position
      newThetaR = newThetaR - (changeR * ((float)(currentPositionR - newPositionR))); //correct by adding difference
    }
    currentPositionR = newPositionR; //reset position
  }
  newTimeR = (millis()/1000.0); //find end time in seconds
  elapsedTime += newTimeR;
  if(turn){
    elapsedTime = 0;
    if(direc == 'L'){
      futurePositionR = -44.4 * angle;
      turn = false;
      //change = true;
    }else if(direc == 'R'){
      futurePositionL = 44.4 * angle;
      turn = false;
      //change = true;
    }
  }
  if(change){
    //futurePositionL = 0;
    //futurePositionR = 0;
    delay(500);
    change = false;
    futurePositionR -= 177 * dist;
    futurePositionL += 177 * dist;
  }
  //wheel diameter = 5.75in, 1 rev = pi*diameter, 3200counts/1rev = 177counts/inch
  //.326587
  
  delay(8 - ((newTimeR - currentTimeR)/1000)); //add delay
  //Serial.print(currentPositionL);
  //Serial.print("\t");
  //Serial.println(newPositionL);
  /*
  if (millis() > 1000) {
    futurePosition = 3000;
  }
  AngVel = 2*3.14*newPosition/(30*(millis()/1000));
  Serial.println(AngVel);
  */
  
  
  
}


//I2C Communication Setup

void receiveEvent(int howMany) {
  Serial.print("Event Received: ");
  uint8_t i = 0;
  while (Wire.available() && (i < msg_size)) { //if data available and less than max size
    msg_recieve[i] = Wire.read(); //read bit of data
    Serial.print(msg_recieve[i]);
    //Serial.print(' ');
    i++; //increment counter
  }
  //msg_recieve[0] gives number 1-4 cooresponding to the quadrant from the event recieved
  Serial.println(' ');
  //Serial.println(i);
  if (msg_recieve[0] == 1) {
    futurePositionL = 800;
    futurePositionR = 800;
  }
  else if (msg_recieve[0] == 2) {
    futurePositionL = 1600;
    futurePositionR = 1600;
  }
  else if (msg_recieve[0] == 3) {
    futurePositionL = 2400;
    futurePositionR = 2400;
  }
  else {
    futurePositionL = 0;
    futurePositionR = 0;
  }
  /*
  if((i == 2)&&(msg_recieve[1] != 0)) { //if 2 bits
    futureTheta = (msg_recieve[1]-1)*1.5707963; //set new theta value
    Serial.print("New position: ");
    Serial.println(" radians.");
   
  }*/
}

void requestEvent () {
  Wire.write((byte) (newPositionL >> 8 * 3) & 0xFF);
  Wire.write((byte) (newPositionL >> 8 * 2) & 0xFF);
  Wire.write((byte) (newPositionL >> 8 * 1) & 0xFF);
  Wire.write((byte) (newPositionL >> 8 * 0) & 0xFF);
}
