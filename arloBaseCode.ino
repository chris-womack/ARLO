#include <Servo.h>

Servo motorRight;
Servo motorLeft;

int data;
long dataAngle;
int cmd;

int speedStop = 1500;
int speedForward = 1300;
int speedBackward = 1600;

const int pingPinFL = 22;
const int pingPinFR = 24;
const int pingPinF = 26;
const int pingPinB = 28;

double distK = 0.534507;

volatile long encoderCountRight;
volatile long encoderCountLeft;

volatile byte timerstatel = 0; // 0 is counting up, 1 is checking
volatile byte timerstater = 0;

volatile long prevCountRight;
volatile long prevCountLeft;

volatile double encoderSpeedRight = 0;
volatile double encoderSpeedLeft = 0;

volatile long lasttimeR = 0;
volatile long lasttimeL = 0;

//when asked for telemetry, we are going to send the 4 sensor readouts and the angular orientation.
//4 bytes for gyroscope data (pitch and roll both as uint16_t) 
//sensor data is a uint8_t, and there are 4 


int right(long angle) {

  motorRight.writeMicroseconds(speedBackward);
  motorLeft.writeMicroseconds(speedForward);
  delay(angle * 15);
  motorRight.writeMicroseconds(speedStop);
  motorLeft.writeMicroseconds(speedStop);
  cmd = 0;
  return 0;
}

int left(long angle) {
  motorLeft.writeMicroseconds(speedBackward);
  motorRight.writeMicroseconds(speedForward);
  delay(angle * 15);
  motorRight.writeMicroseconds(speedStop);
  motorLeft.writeMicroseconds(speedStop);
  cmd = 0;
  return 0;
}

int forward(long distance) {
  double lerror = 0;
  double rerror = 0;
  double preverrorL = 0;
  double preverrorR = 0;
  double lintegral = 0;
  double rintegral = 0;
  float propk = 1; //1
  float intk = 0.35; //.5
  float derk = 0.25; //0.25
  int speedRight = speedForward ; //was speedForward - 45
  int speedLeft = speedForward;
  double encoderValue = speedForward * (-0.13125) + 190.3125; //desired encoder value
  int count = 0;

  motorRight.writeMicroseconds(speedRight);
  motorLeft.writeMicroseconds(speedLeft);
  Serial.println("DESIRED SPEED");
  Serial.println(encoderValue);
  while (1) {
    if (encoderCountLeft != encoderCountRight) {
      // delay(50); //For some reason this needs to be this long
      delay(50);

      lerror = -(encoderSpeedLeft - encoderValue);

      double lpropterm = lerror * propk;
      double lintterm = lintegral * intk;
      double lderterm = ((lerror - preverrorL) / (50)) * derk;
      double lwriteValue = ((encoderValue + lpropterm + lintterm + lderterm) < 190.3125 ) ? (encoderValue + lpropterm + lintterm + lderterm) : 190.3124;
      motorLeft.writeMicroseconds((lwriteValue - 190.3125) / -0.13125);
      lintegral += lerror;
      preverrorL = lerror;

      rerror = -(encoderSpeedRight - encoderValue);
      double rpropterm = rerror * propk;
      double rintterm = rintegral * intk;
      double rderterm = ((rerror - preverrorR) / (50)) * derk;
      double rwriteValue = ((encoderValue + rpropterm + rintterm + rderterm) < 179.85 ) ? (encoderValue + rpropterm + rintterm + rderterm) : 179.85;
      motorRight.writeMicroseconds((rwriteValue - 179.85) / -0.12403);
      rintegral += rerror;
      preverrorR = rerror;

      if (distK * (double)encoderCountLeft > (double) distance)
        break;
      if (distK * (double)encoderCountRight > (double) distance)
        break;
    }
    else {
      if (distK * (double)encoderCountLeft > (double) distance)
        break;
      if (distK * (double)encoderCountRight > (double) distance)
        break;
    }
  }
  motorRight.writeMicroseconds(speedStop);
  motorLeft.writeMicroseconds(speedStop);
  encoderCountRight = 0;
  encoderCountLeft = 0;
  cmd = 0;
  return 0;
}



int backward(long distance) {
  int error = 0;
  int speedRight = speedBackward;
  int speedLeft = speedBackward + 37;
  int count = 0;
  motorRight.writeMicroseconds(speedRight);
  motorLeft.writeMicroseconds(speedLeft);
  while (1) {
    if (encoderCountLeft != encoderCountRight) {
      delay(50); //For some reason this needs to be this long
      motorRight.writeMicroseconds(speedRight + (int)((float)error * 10));
      motorLeft.writeMicroseconds(speedLeft - (int)((float)error * 10));
      error = encoderCountLeft - encoderCountRight;
      if (distK * (double)encoderCountLeft > (double) distance)
        break;
      if (distK * (double)encoderCountRight > (double) distance)
        break;
    }
    else {
      if (distK * (double)encoderCountLeft > (double) distance)
        break;
      if (distK * (double)encoderCountRight > (double) distance)
        break;
    }
  }
  motorRight.writeMicroseconds(speedStop);
  motorLeft.writeMicroseconds(speedStop);
  encoderCountRight = 0;
  encoderCountLeft = 0;
  cmd = 0;
  return 0;
}

void setup () {
  Serial.begin(9600);//for testing purposes we will remove parity //,SERIAL_8E1);
  Serial.setTimeout(50); // this is to make parseInt not so ridiculously slow
  //initialize timers TIMER


  cli(); // disable global interrupts

  TCCR3A = 0; // zero out TCCR3A reg
  TCCR3B = 0; // zero out TCCR3B reg
  // TCCR3A is operating in normal
  // TCCR3B |= (1 << CS32) | (1 << CS30); // set prescaler for timer3 (1024)
  TCCR3B |= (1 << CS32); //set prescaler for timer3 (256)
  TCNT3 = 0; // initialize count
  TIMSK3 = 0; // zero out interrupt reg
  TCCR4A = 0;
  TCCR4B = 0;
  // TCCR4B |= (1 << CS42) | (1 << CS40);
  TCCR4B |= (1 << CS42);
  TCNT4 = 0;
  TIMSK4 = 0;
  sei(); // enable global interrupts:
  motorRight.attach(13);
  motorLeft.attach(12);
  attachInterrupt(0, encoderHandlerRight, RISING);
  attachInterrupt(1, encoderHandlerLeft, RISING);
}

void loop() {
  data = 0;
  encoderCountRight = 0;
  encoderCountLeft = 0;

  // Check buffer for commands
  while (Serial.available() > 0) {
    cmd = Serial.read();
    data = Serial.parseInt(); //SLOW !!!!!
  }

  // Logic for commands
  switch (cmd) {
    case 'R':
      Serial.println(cmd);
      Serial.println(data);
      right(data);
      break;
    case 'L':
      Serial.println(cmd);
      Serial.println(data);
      left(data);
      break;
    case 'F':
      forward(data);
      break;
    case 'B':
      backward(data);
      break;
    default:
      break;
  }
}

void encoderHandlerRight() {

  encoderCountRight++;
  if (timerstater) {
    double t = TCNT3;
    if (t > (320 * 4)) // cut off for our high frequnecy filter. Chosen somewhat arbitrarily.
      encoderSpeedRight = ((double)(1 / ((double)t))) * 16000 * 4; //speed in ticks/sec
    //Serial.print("Right speed: ");
    Serial.print(encoderSpeedRight, 20);
    Serial.print("\n");
  }
  else
    TCNT3 = 0;
  timerstater = !timerstater;
}

void encoderHandlerLeft() {
  encoderCountLeft++;
  if (timerstatel) {
    double t = TCNT4;
    if (t > (320 * 4))
      encoderSpeedLeft = ((double)(1 / ((double)t))) * 16000 * 4; //speed in ticks/sec
    /*  Serial.print("Left speed: ");
      Serial.print(encoderSpeedLeft, 20);
      Serial.print("\n"); */
  }
  else
    TCNT4 = 0;
  timerstatel = !timerstatel;
}

