#include <Wire.h>

// Initialize variables from the RPi
bool markerFound = false;
float angle = 0;
float dist = 0;

void setup() {
  // Join I2C bus as slave with address 8
  Wire.begin(0x08);
  
  Wire.onRequest(requestEvent);
  
  // Call receiveEvent when data received                
  Wire.onReceive(receiveEvent);
  
  Serial.begin(115200);
}

// Function that executes whenever data is received from master
void receiveEvent(int howMany) {
  if (howMany < 5){
    while (Wire.available()) {
      Wire.read(); // Clear Buffer
    }
  }
  else{
    // Initialize variables to read in the incoming bytes
    uint8_t buff[5];
    uint8_t i = 0;
    
    // IDK why but I had to read an extra byte every time
    Wire.read();

    // Set the requested data
    while (Wire.available()) { // loop through all but the last
      buff[i++] = (uint8_t) Wire.read(); // receive byte as a character
    }
  
    markerFound = buff[0];
   
    // Cool pointer stuff
    int16_t * int_buff = (void*) &buff[1];
    angle = *int_buff/100.0;
//    if(abs(angle) < .5 and angle != 0){
//      angleMoved = true;
//    }
    int_buff = (void*) &buff[3];
    dist = *int_buff/100.0;
    dist = dist / 2.54;
  }
}

void requestEvent(){
  Wire.write(2);
  
}

void loop() {
  Serial.println(markerFound);
  Serial.println(angle);
  Serial.println(dist);
  Serial.println("\n\n");
  delay(100);
}
