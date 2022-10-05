#include <DualMC33926MotorShield.h>
DualMC33926MotorShield md;
#include <Wire.h>
#include <Encoder.h>
Encoder myEnc(2, 5); //set encoder pins for motor

#define msg_size 32 //define size of max recieve message
#define resetn 4 //define reset pin (active low)
#define PWM_out 9 //define output PWM pin
#define directionPin 7 //define pin controlling direction
uint8_t msg_recieve[msg_size];

void setup() {
  Serial.begin(115200); //set baud rate
  pinMode(PWM_out, OUTPUT); //assign output PWM pin
  
  digitalWrite(resetn, HIGH); //toggle reset pin
  analogWrite(PWM_out, 0); //initialize PWM
  md.init(); //initialize motor shield
  analogWrite(PWM_out,0); //initialize PWM
  Wire.begin(0x8); //join I2C bus as slave with address 8
  Wire.onReceive(receiveEvent); //call recieveEvent when data received
  Wire.onRequest(requestEvent); //call requestEvent when data requested

}

//Initialize position and motor controller variables
long currentPosition = -999; //initialize current Position
long newPosition = 0; //initialize new Position
long desiredPosition = 0; //initialize desired Position from Aruco marker
double change = (2*3.14159265)/3200; //delta value (2pi/3200)
double currentTheta = 0; //initialize current theta value
double newTheta = 0; //initialize the new theta value
double voltage = 0; //initialize voltage level
double currentVoltage = 8.0; //set voltage cap
int PWM = 0; //initialize PWM output to motor
double currentTime = 0; //initialize current time
double newTime = 0; //initialize the new time
double error = 0; //initialize error value
double Kp = 8; //initialize Kp
double Ki = 0.0008; //initialize Ki
double currentIntegral = 0; //initialize current integral term
double newIntegral = 0; //initialize new integral term
double futureTheta = 0; //initialize future theta term
double futurePosition = 0; //initialize future position term
int AngVel = 0;

void loop() {
  //controller variable equations
  currentTime = (millis()/1000.0); //find current time in seconds
  currentTheta = newTheta; //reset the new theta with the old theta
  newIntegral = currentIntegral + (error*change); //In = Io + (e*d)*I
  currentIntegral = newIntegral; //reset old integral term
  //futurePosition = futureTheta * 509; //x = theta * P
  error = futurePosition - newPosition; //e = x1 - x0
  voltage = Kp * (error * change) + (Ki * newIntegral); //v = Kp * (e*d) + (Ki * In)
  PWM = (voltage/currentVoltage)*255; //set PWM output
  if (PWM > 255) { 
    PWM = 255; //if above max value, set at cap
  }
  else if (PWM < -255) {
    PWM = -255; //if below min value, set at floor
  }
  if (PWM >= 0) {
    digitalWrite(directionPin, LOW); //set direction if positive
  }
  if (PWM < 0) {
    digitalWrite(directionPin, HIGH); //set direction if negative
  }
  analogWrite(PWM_out, abs(PWM)); //set PWM pulse absolute value
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
  Serial.println(newPosition);
  // Speed limit
  newPosition = myEnc.read(); //read encoder value
  //Serial.println(newPosition); xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  if (newPosition != currentPosition) { //if detected change in position
    if (newPosition > currentPosition) { //if increased position
      newTheta = newTheta + (change * ((float)(newPosition - currentPosition))); //correct by subtracting difference
    }
    else { //if decreased position
      newTheta = newTheta - (change * ((float)(currentPosition - newPosition))); //correct by adding difference
    }
    currentPosition = newPosition; //reset position
  }
  newTime = (millis()/1000.0); //find end time in seconds
  delay(8 - ((newTime - currentTime)/1000)); //add delay
  //Serial.print(currentPosition);
  //Serial.print("\t");
  //Serial.println(newPosition);
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
    futurePosition = 800;
  }
  else if (msg_recieve[0] == 2) {
    futurePosition = 1600;
  }
  else if (msg_recieve[0] == 3) {
    futurePosition = 2400;
  }
  else {
    futurePosition = 0;
  }
  /*
  if((i == 2)&&(msg_recieve[1] != 0)) { //if 2 bits
    futureTheta = (msg_recieve[1]-1)*1.5707963; //set new theta value
    Serial.print("New position: ");
    Serial.println(" radians.");
   
  }*/
}

void requestEvent () {
  Wire.write((byte) (newPosition >> 8 * 3) & 0xFF);
  Wire.write((byte) (newPosition >> 8 * 2) & 0xFF);
  Wire.write((byte) (newPosition >> 8 * 1) & 0xFF);
  Wire.write((byte) (newPosition >> 8 * 0) & 0xFF);
}
