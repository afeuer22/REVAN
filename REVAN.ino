#include <AccelStepper.h>
#include <Servo.h>
int IDs[] = {34, 36, 38, 40, 42, 44};
int LSs[] = {A0,A1,A2,A3,A4,A5};
Servo myservo;
#define IDIN A6
#define EE 8
long upperlimit = 0;
long lowerlimit =0;
String Config ="R";
unsigned long lastSend = 0;
const int sendInterval = 50;  
const int steps_per_rotation = 200;

float microstep= 8;
float deg_per_step = 360/(steps_per_rotation*microstep);

AccelStepper Joints[6] = {
AccelStepper(AccelStepper::DRIVER, 2, 22), 
AccelStepper(AccelStepper::DRIVER, 3, 24), 
AccelStepper(AccelStepper::DRIVER, 4, 26), 
AccelStepper(AccelStepper::DRIVER, 5, 28), 
AccelStepper(AccelStepper::DRIVER, 6, 30), 
AccelStepper(AccelStepper::DRIVER, 7, 32), 
};

float targetSpeeds[6] = {0,0,0,0,0,0};

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 6; i++) {
    pinMode(IDs[i], OUTPUT);
    pinMode(LSs[i], INPUT);
    digitalWrite(IDs[i],LOW);
    digitalWrite(LSs[i],LOW);
  }
  pinMode(IDIN,INPUT);
  myservo.attach(EE);

    for (int i = 0; i < 6; i++) {
    Joints[i].setMaxSpeed(4000);
    Joints[i].setAcceleration(200);
  }
}


void loop() {

//read serial monitor
  if (Serial.available()) {
    Serial.flush();
    String incoming = Serial.readStringUntil('\n'); // Read until newline
    incoming.trim();                         // Remove spaces/newlines
    if (incoming.length() == 0)
      return;

    handleCommand(incoming);
  }

//Check J limit or current pos based on 50ms interval
    if (millis() - lastSend >= sendInterval) {
      lastSend = millis();
      //update joint angle if joint is moving
      for (int j = 0; j < 6; j++) {
         if(targetSpeeds[j]!=0){
         getAngles(j);
      }
      long pos = Joints[j].currentPosition();
      //stop motor and send error at joint limit
      if ((pos >= getupperlimit(j) && targetSpeeds[j] > 0) || (pos <= 0 && targetSpeeds[j] < 0)) {
        targetSpeeds[j] = 0;           // stop
        Serial.println("E2");
        Serial.flush();
      }

    }
  }

//move joints based on set speed
  for (int i = 0; i < 6; i++) {
   Joints[i].setSpeed(targetSpeeds[i]);  
   Joints[i].runSpeed();                 // <<< MUST BE CALLED CONSTANTLY
  }
}


void handleCommand(String cmd) {
  if (checkReset()) {
    // 1. Flush Arduino-side serial buffer
    while (Serial.available() > 0) Serial.read();
    return;
  }

  char motorChar = cmd.charAt(0);
  char direction = cmd.charAt(1);

  // Handle special commands
  if (!isDigit(motorChar)) {

    if (motorChar == 'C') { Calibrate(); return; }
    if (motorChar == 'R') { Refresh(); return; }
    if (motorChar == 'G'){Gripper(direction); return;}
    if(motorChar =='P'){
      for(int j=0;j<6;j++){
        getAngles(j);
      }
    }

    return;
  }

  int motorIndex = motorChar - '0';  // convert char to number
  motorIndex -= 1;                       // convert to 0–5 index

  AccelStepper &m = Joints[motorIndex];

  if (direction == '+') {
    // Move forward continuously
    targetSpeeds[motorIndex] = +4000;
  }
  else if (direction == '-') {
    // Move backward continuously
    targetSpeeds[motorIndex] = -4000;
  }
  else if (direction == '0') {
    // Stop motion
    targetSpeeds[motorIndex] = 0;
  }

}
void Refresh(){ 
  digitalWrite(EE,HIGH);
  delay(100);
  if(analogRead(IDIN) <800){
    Serial.println("E1");
    Serial.flush(); 
    digitalWrite(EE,LOW);
    return;
  }
  else{
    digitalWrite(EE,LOW);
    for(int i=3;i<6;i++){
      digitalWrite(IDs[i],HIGH);
      delay(50);
      if(analogRead(IDIN)>200) {
        Config= Config + (i+1);
        digitalWrite(IDs[i],LOW);
      }
      else {
        digitalWrite(IDs[i],LOW);
        break;
      }
    }
    Serial.println(Config);
    Serial.flush();
  }
}

void Calibrate(){
  for (int i=0;i<(Config.length()+2);i++){
     targetSpeeds[i] = 1000;
     Joints[i].setCurrentPosition(0);	
      while(true){
        if (checkReset()) {
          while (Serial.available() > 0) Serial.read();
          return;
        }
        if(analogRead(LSs[i]) >900)
          break;
        if(Joints[i].currentPosition()>getupperlimit(i)){
          targetSpeeds[i] = -1000;
        }
        if(Joints[i].currentPosition()<0||Joints[i].currentPosition()>getupperlimit(i)){
          Serial.println("E3");
          Serial.flush();
          return;
        } 
        Joints[i].setSpeed(targetSpeeds[i]);   
        Joints[i].runSpeed(); 
      }
      delay(100);
      Joints[i].setCurrentPosition(0);	
      Serial.print("Joint ");
      Serial.print(i+1);
      Serial.println(" calibrated");
      Serial.flush();
  }
  Serial.println("D");
  Serial.flush();
}

void Gripper(char dir){
  if(dir == "+"){
    myservo.write(170);
  }
  else{
    myservo.write(10);
  }
}


long getupperlimit(int i){
  upperlimit= steps_per_rotation*microstep*getGearRatio(i);
  return upperlimit;
}    

int getGearRatio(int i){
  if (i<2){
    //return 42;
    return 30;
  }
  else{
    return 30;
  }
}

bool checkReset(){
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'X') {
    Serial.println("ABORTED");
     for (int i=0;i<6;i++){
        targetSpeeds[i] =0;
        Joints[i].setSpeed(targetSpeeds[i]);  
     }
    return true;           // exit function
    }
  }
  return false;
}

void getAngles(int j){
  long steps = Joints[j].currentPosition();
  float angle = (steps * deg_per_step) / getGearRatio(j) ; // degrees-per-step
  Serial.print("P");
  Serial.print(j + 1); // 1-based joint index
  Serial.print(" ");
  Serial.println(angle); // 1 decimal place
}
