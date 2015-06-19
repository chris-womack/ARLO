#include <Wire.h>
#include <SharpIR.h>


#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define ir1 A2 //Front-Left IR
#define ir2 A1 //Front-Center IR
#define ir3 A0 //Front-Right IR

#define model 1080



int Addr = 105;                 // I2C address of gyro
int x, y, z;
int j; 
int xsum,ysum,zsum; 
float truex, truey, truez;
float pitch, roll, yaw, pitch_output, roll_output;

char user_input;
const int pingPin1 = 9; //Front-Left Ultra Sonic
const int pingPin2 = 10; //Front-Center Ultra Sonic
const int pingPin3 = 11; //Front-Right Ultra Sonic
const int pingPin4 = 6; //Rear Sensor

unsigned int duration1, duration2, duration3, duration4;
int cm1, cm2, cm3, cm4, output_dis1, output_dis2, output_dis3, output_dis4;
boolean done=false;


SharpIR sharp1(ir1, 25, 93, model);
SharpIR sharp2(ir2, 25, 93, model);
SharpIR sharp3(ir3, 25, 93, model);

void setup(){
  Wire.begin();
  Serial.begin(9600);
  pinMode (ir1, INPUT);
  pinMode (ir2, INPUT);
  pinMode (ir3, INPUT);
  writeI2C(CTRL_REG1, 0x1F);    // Turn on all axes, disable power down
  writeI2C(CTRL_REG3, 0x08);    // Enable control ready signal
  writeI2C(CTRL_REG4, 0x80);    //250 dps
  delay(100);                   // Wait to synchronize 
  //pinMode (ir4, INPUT);
  for(int i = 0; i<= 1000; i++){ // IMPORTANT NOTE: We are only calculating R0 at the beginning of the program. THIS COULD POTENTIALLY BEA PROBLEM, AS THE LEVEL OF NOISE CHANGES. May need to retake this every so often. 
      getGyroValues(); 
      xsum+=x; 
      ysum+=y; 
      zsum+=z; 
  }
      xsum/=100; 
      ysum/=100;
      zsum/=100;

}



void loop() {
    getGyroValues();
    //x, y and z contain raw gyro readings
    truex = ((x - xsum)*0.00875); //from data sheet, sensitivity for 250 dps 
    truey = ((y - ysum)*0.00875); 
    truez = ((z - zsum)*0.00875); 
    truex = ( abs(truex) < 5 ? 0 : truex ); 
    truey = ( abs(truey) < 5 ? 0 : truey ); 
    truez = ( abs(truez) < 5 ? 0 : truez ); 
    roll += truex*0.013; 
    pitch += truey*0.013; 
    yaw += truez*0.013;  
    
  user_input = Serial.read();
    if (user_input == 'I'){
  pinMode(pingPin1, OUTPUT);          // Set pin to OUTPUT
  digitalWrite(pingPin1, LOW);        // Ensure pin is low
  delayMicroseconds(2);
  digitalWrite(pingPin1, HIGH);       // Start ranging
  delayMicroseconds(5);              //   with 5 microsecond burst
  digitalWrite(pingPin1, LOW);        // End ranging
  pinMode(pingPin1, INPUT);           // Set pin to INPUT
  duration1 = pulseIn(pingPin1, HIGH); // Read echo pulse
  delay(10);
  
  pinMode(pingPin2, OUTPUT);          // Set pin to OUTPUT
  digitalWrite(pingPin2, LOW);        // Ensure pin is low
  delayMicroseconds(2);
  digitalWrite(pingPin2, HIGH);       // Start ranging
  delayMicroseconds(5);              //   with 5 microsecond burst
  digitalWrite(pingPin2, LOW);        // End ranging
  pinMode(pingPin2, INPUT);           // Set pin to INPUT
  duration2 = pulseIn(pingPin2, HIGH); // Read echo pulse
  delay(10);
  
  pinMode(pingPin3, OUTPUT);          // Set pin to OUTPUT
  digitalWrite(pingPin3, LOW);        // Ensure pin is low
  delayMicroseconds(2);
  digitalWrite(pingPin3, HIGH);       // Start ranging
  delayMicroseconds(5);              //   with 5 microsecond burst
  digitalWrite(pingPin3, LOW);        // End ranging
  pinMode(pingPin3, INPUT);           // Set pin to INPUT
  duration3 = pulseIn(pingPin3, HIGH); // Read echo pulse
  delay(10);
  
  pinMode(pingPin4, OUTPUT);          // Set pin to OUTPUT
  digitalWrite(pingPin4, LOW);        // Ensure pin is low
  delayMicroseconds(2);
  digitalWrite(pingPin4, HIGH);       // Start ranging
  delayMicroseconds(5);              //   with 5 microsecond burst
  digitalWrite(pingPin4, LOW);        // End ranging
  pinMode(pingPin4, INPUT);           // Set pin to INPUT
  duration4 = pulseIn(pingPin4, HIGH); // Read echo pulse
  delay(10);
  
//////////////////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------------------------//
  unsigned long pepe1=millis();  // takes the time before the loop on the library begins
  
  

  int dis1=sharp1.distance();  // this returns the distance to the object you're measuring
  int dis2=sharp2.distance();  // this returns the distance to the object you're measuring
  int dis3=sharp3.distance();  // this returns the distance to the object you're measuring
  //int dis4=sharp4.distance();  // this returns the distance to the object you're measuring

  //----------------------------------ERROR CHECK SECTION---------------------------------------------------//
  cm1 = duration1 / 29 / 2;  // Convert to cm from inches
  cm2 = duration2 / 29 / 2;  // Convert to cm from inches
  cm3 = duration3 / 29 / 2;  // Convert to cm from inches
  cm4 = duration4 / 29 / 2;  // Convert to cm from inches
  
  output_dis1 = cm1;
  output_dis2 = cm2;
  output_dis3 = cm3;
  //output_dis4 = cm4;
  
  /*if(cm1 > 250){
    output_dis1= dis1;
   }
  if(cm2 > 250){
    output_dis2= dis2;
   }
  if(cm3 > 250){
    output_dis3= dis3;
   }*/
//----------------------------------------------------PRINT THE DISTANCES---------------------------------// 
  pitch_output= pitch;
  roll_output= roll;
  if(abs(pitch) < 8){
    pitch_output= 0;
  }
  if(abs(roll) < 8){
    roll_output= 0;
  }
  Serial.print(pitch_output); //Pitch
  Serial.print(" ");
  Serial.print(roll_output); //Roll
  Serial.print(" ");
  Serial.print(output_dis1); //Front Left Sensor
  Serial.print(" ");
  Serial.print(output_dis2); // Front Center Sensor
  Serial.print(" ");
  Serial.print(output_dis3); //Front Right Sensor 
  Serial.print(" ");
  Serial.println(cm4); // Rear Sensor
    
    }
delay(10);
}

void getGyroValues () {
  byte MSB, LSB;

  MSB = readI2C(0x29);
  LSB = readI2C(0x28);
  x = ((MSB << 8) | LSB);

  MSB = readI2C(0x2B);
  LSB = readI2C(0x2A);
  y = ((MSB << 8) | LSB);

  MSB = readI2C(0x2D);
  LSB = readI2C(0x2C);
  z = ((MSB << 8) | LSB);
}

int readI2C (byte regAddr) {
    Wire.beginTransmission(Addr);
    Wire.write(regAddr);                // Register address to read
    Wire.endTransmission();             // Terminate request
    Wire.requestFrom(Addr, 1);          // Read a byte
    while(!Wire.available()) { };       // Wait for receipt
    return(Wire.read());                // Get result
}

void writeI2C (byte regAddr, byte val) {
    Wire.beginTransmission(Addr);
    Wire.write(regAddr);
    Wire.write(val);
    Wire.endTransmission();
}


