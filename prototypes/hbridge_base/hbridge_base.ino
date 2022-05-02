#define REVERSE_LEFT_MOTOR false 
#define REVERSE_RIGHT_MOTOR false
#define AIN1 11
#define AIN2 10
#define SLP 12
#define BIN1 9
#define BIN2 5

void setup() {
  // set up h-bridge
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  pinMode(SLP, OUTPUT);
  digitalWrite(SLP, HIGH);

  // initialize serial
  Serial.begin(9600);
}

void loop() {
  setSpeeds(0, 0);
  delay(1000);
  setSpeeds(255, 255);
  delay(1000);
  setSpeeds(-255, 255);
  delay(1000);
  setSpeeds(-255, -255);
  delay(1000);
  setSpeeds(255, -255);
  delay(1000);
}

void setSpeeds(float leftSpeed, float rightSpeed){
  setHBridge(AIN1, AIN2, abs(rightSpeed), (rightSpeed < 0) != REVERSE_RIGHT_MOTOR /*drive pin 2 if the speed is negative xor it should be reversed*/);
  setHBridge(BIN1, BIN2, abs(leftSpeed), (leftSpeed < 0) != REVERSE_LEFT_MOTOR /*drive pin 2 if the speed is negative xor it should be reversed*/);
}

void setHBridge(int in1, int in2, int speed, bool drivePin1){
  // pwm the high pin
  if(drivePin1){
    digitalWrite(in2, LOW);
    analogWrite(in1, speed);
  } else {
    digitalWrite(in1, LOW);
    analogWrite(in2, speed);
  }
}