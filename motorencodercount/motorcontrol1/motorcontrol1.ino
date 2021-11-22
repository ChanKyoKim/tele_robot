#include <Arduino.h>
#include <TimerOne.h>
#define speedPin 6
#define normalRotation 7
#define backRotation 8
//motor encoder
long pinA = 21;
long pinB = 20;
volatile long count = 0;
long angvel = 0;
long precount = 0;
long angacc = 0;
long prevel = 0;
//encoder
long encoderPin1 = 2; //A
long encoderPin2 = 3; //B
volatile long angle = 0;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
//motor command
int motorComm = 1;

void setup() {
  Serial.begin(9600);
  //encoder, motor rotation
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  pinMode(speedPin, OUTPUT);
  pinMode(normalRotation, OUTPUT);
  pinMode(backRotation, OUTPUT);
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
  //motor encoder
  pinMode(pinA, INPUT);
  attachInterrupt(2, Count, CHANGE);
  Timer1.initialize(50000);
  Timer1.attachInterrupt(Anginf);
}

void loop() {
  // Serial.print("Motor position"); Serial.println(count);
  Serial.print("Motor velocity"); Serial.println(angvel);
  Serial.print("Motor acceleration"); Serial.println(angacc);
  Serial.println(angle);
  if (motorComm == 1) {
    Serial.println("Motor Run");
    MotorNR();
  }
  else if (motorComm == -1) {
    Serial.println("Motor Stop");
    MotorStop();
    delay(10000);
    MotorBR();
    delay(7 0);
    MotorStopcont();
  }

}
void Count() {
  count ++;
}
void Anginf() {
  angvel = count - precount;
  precount = count;
  angacc = angvel - prevel;
  prevel = angvel;
  if (angacc < -2) motorComm = -1;
}
void MotorNR() {
  digitalWrite(speedPin, HIGH);
  digitalWrite(normalRotation, HIGH);
  digitalWrite(backRotation, LOW);
}
void MotorBR() {
  digitalWrite(speedPin, HIGH);
  digitalWrite(normalRotation, LOW);
  digitalWrite(backRotation, HIGH);
}
void MotorStop() {
  digitalWrite(speedPin, HIGH);
  digitalWrite(normalRotation, LOW);
  digitalWrite(backRotation, LOW);
}
void updateEncoder() {
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
  lastEncoded = encoded; //store this value for next time
  angle = -encoderValue * 360 / 2048;
}
void MotorStopcont() {
  digitalWrite(speedPin, HIGH);
  digitalWrite(normalRotation, LOW);
  digitalWrite(backRotation, LOW);
  delay(100000);
}
