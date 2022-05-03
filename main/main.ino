#include <Servo.h>

#define REVERSE_A_MOTOR false 
#define REVERSE_B_MOTOR false
#define AIN1 11
#define AIN2 10
#define SLP 12
#define BIN1 9
#define BIN2 5

#define SERVO_PIN 8
#define ARM_BUTTON 7
#define ARM_LED 6
#define PHOTO_PIN 2

enum RaceState {
  waitForArm,
  armed,
  raceLowGear,
  raceHighGear,
  test // used when testing a new feature, should not be set in normal execution
}
enum Gear{
  high = 10, // the enum does double duty by serving as both a human-readable name
  low = 170, // and a map from gear name to servo value for that gear
  neutral = 0
}
struct MotorState{
  int motorSpeed;
  bool isCoastMode = true;
};
bool test = false;
RaceState state;
Gear currentGear;
MotorState currentMotorState;

long gearChangeTime = 2000; // not const b/c can be reconfig'd over serial
long brakeTime = 5000; // ^^^^^^^^
long raceStartTime;

Servo shifter;

String inString = "";    // string to hold serial input

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
  // set up sensors + arm LED
  pinMode(ARM_LED, OUTPUT);
  pinMode(ARM_BUTTON, INPUT_PULLUP);
  pinMode(PHOTO_PIN, INPUT_PULLUP);
  // set up shifter servo
  shifter.attach(SERVO_PIN);

  // initialize serial
  Serial.begin(9600);

  // set race state
  state = waitForArm;
  // shift to low gear
  changeGear(low);
}

void loop() {
  handleState();
  // Tunable shift and brake timing over serial:
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    } else if (inChar == 'g') {
      // it's the gear change time
      gearChangeTime = inString.toInt();
      Serial.print("Changed gear shift time to ");
      Serial.println(gearChangeTime);
    } else if (inChar == 'b') {
      // it's the brake time
      brakeTime = inString.toInt();
      Serial.print("Changed brake time to ");
      Serial.println(brakeTime);
    } else {
      Serial.print("Got unrecognized char ");
      Serial.println((char)inChar);
    }
}

void handleState(){
  // at some point I should write a generalized StateMachine lib and make this neater
  switch(state){
    case waitForArm:
      digitalWrite(ARM_LED, LOW); // keep armed LED off
      setMotorState(MotorState{0, false}); // keep motors braked
      // exit condition: arm button pressed
      if(digitalRead(ARM_BUTTON) == LOW){
        state = armed;
        digitalWrite(ARM_LED, HIGH); // turn armed LED on
        // also shift into low here to make sure
        changeGear(low);
      }
      break;
    case armed:
      // exit condition: photointerruptor is light-on, so trigger on HIGH (empty)
      if(digitalRead(PHOTO_PIN == HIGH)){
        state = raceLowGear;
        // set the start time so we can shift and brake relative to it
        raceStartTime = millis();
        // start the motors
        setMotorState(MotorState{255})
      }
      break;
    case raceLowGear:
      // exit condition: time elapsed > gearChangeTime
      long timeElapsed = millis() - raceStartTime
      if(timeElapsed > gearChangeTime){
        state = raceHighGear;
        changeGear(high);
      }    
      break;
    case raceHighGear:
      // exit condition: time elapsed > brakeTime
      long timeElapsed = millis() - raceStartTime
      if(timeElapsed > brakeTime){
        state = waitForArm;
        changeGear(low);
      }  
      break;
    case test:
      break;
  }
}

void changeGear(Gear targetGear){
  // this uses delays and everything else is delay-less
  // it's not pretty but imo it's better than having a global isChangingGears flag
  // because this method must be blocking, while the other states are more tolerant of changes
  // anyways, to shift a gear we:
  // 0. make sure we aren't already here
  if(currentGear == targetGear){
    return;
  }
  currentGear = targetGear;
  // 1. coast the motors
  setMotorState(MotorState{0, true});
  // 2. wait a little for the h-bridge to disengage
  delay(2);
  // 3. move servo to correct position for gear
  shifter.write(targetGear);
  // 4. wait for the servo to shift positions
  delay(250);
  // 5. restore motor speeds
  setMotorState(currentMotorState)
}

void setMotorState(MotorState newState){
  // make sure to set currentMotorState ouside of here because changeGear relies
  // on this not touching it
  if(newState.motorSpeed == 0){
    // special handling for stop mode
    if(newState.isCoastMode){
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
    } else {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, HIGH);
    }
  } else {
    setHBridge(AIN1, AIN2, abs(newState.motorSpeed), (newState.motorSpeed < 0) != REVERSE_A_MOTOR /*drive pin 2 if the speed is negative xor it should be reversed*/);
    setHBridge(BIN1, BIN2, abs(newState.motorSpeed), (newState.motorSpeed < 0) != REVERSE_B_MOTOR /*drive pin 2 if the speed is negative xor it should be reversed*/);
  }
}

void setHBridge(int in1, int in2, int motorSpeed, bool drivePin1){
  // pwm the high pin
  if(drivePin1){
    digitalWrite(in2, LOW);
    analogWrite(in1, motorSpeed);
  } else {
    digitalWrite(in1, LOW);
    analogWrite(in2, motorSpeed);
  }
}
