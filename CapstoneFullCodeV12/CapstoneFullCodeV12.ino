//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////          MAE162E GROUP 5 CODE            ///////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//V12
//Competition-grade
//Faster than V11


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////          INCLUDE LIBRARIES           ///////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include "DeviceDriverSet_xxx0.h"
#include <Servo.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////          VARIABLE/PIN DEFINITIONS           ////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define IR_SENSOR_RIGHT 30
#define IR_SENSOR_LEFT 32
#define EXT_IR_SENSOR_RIGHT 34
#define EXT_IR_SENSOR_LEFT 23
#define trigPinL 24
#define echoPinL 22
#define trigPinR 28
#define echoPinR 26


//IR Remote
DeviceDriverSet_Motor myMotors;
DeviceDriverSet_IRrecv IRremote;
uint8_t IRrecv_button;


//Motor Control
int leftWheels = 6;
int rightWheels= 5;
int leftWheelsDir = 8;
int rightWheelsDir = 7;
int motorsEnable = 3;
int speed1 = 70 + 25;
int speed2 = 40;
int speed3 = 75;
int speed4 = 110;
int speed5 = 70;


//ultrasonic parameters
float distanceR, durationR;
float distanceL, durationL;
float stoppingDistance = 3;
float stoppingDistance2 = 10;
float stoppingDistance3 = 13;


//IR Sensors
int rightIRSensorValue = 0;
int leftIRSensorValue = 0;
int EXTrightIRSensorValue = 0;
int EXTleftIRSensorValue = 0;


//Line Tracking and Branch Control
bool stopTracking1 = false;
bool stopTracking2 = true;
bool BranchDetect1 = false;
bool leavingFirstBranches = false;
bool lastBranch1 = false;
int branchNumber1 = 0;
int branchCounter1 = 0;
int branchesLeft1 = 0;
int branchNumber2 = 0;
int branchCounter2 = 0;
int branch1isOnTheLeft = 0;
int shelfPosition1 = 0;
int shelfPosition2 = 0;


//moving obstacle
int delayCounter = 0;
int delayCounter2 = 0;


//IR remote
bool firstButtonPressed = false;
bool secondButtonPressed = false;
int enterPressed1 = 0;
int enterPressed2 = 0;
bool RUN_PROGRAM = false;
int shutoffPressed = 0;
bool stage1 = false;
bool stage2 = false;
bool stage2b = false;
bool stage2c = false;
bool stage3 = false;
bool stage4 = false;
bool stage4b = false;
bool stage4c = false;
bool stage5 = false;
bool stage6 = false;
bool stage6b = false;
bool stage6c = false;
bool stage6d = false;
bool stage6e = false;
bool stage7 = false;
bool stage7b = false;
bool stage7c = false;
bool stage7d = false;
bool stage8 = false;
bool stage9 = false;
bool stage9b = false;
bool stage9c = false;
bool stage10 = false;
bool stage11 = false;
bool switch1 = true;
bool switch2 = true;
bool switch3 = true;
bool switch4 = true;
bool switch5 = true;


//Pickup
bool initiatePickup = false;
bool pickupComplete = false;


//Gripper and Lift
int gripperMotorPin = 10;
int liftMotorPin = 11;
Servo gripperMotor;
Servo liftMotor;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////          VOID SETUP           //////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////










void setup() {
  Serial.begin(9600);


  //IR initialization
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(EXT_IR_SENSOR_RIGHT, INPUT);
  pinMode(EXT_IR_SENSOR_LEFT, INPUT);


  //ultrasonic initialization
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);


  //IR Remote initialization
  IRremote.DeviceDriverSet_IRrecv_Init();
  myMotors.DeviceDriverSet_Motor_Init();


  //Gripper and lift initialization
  pinMode(gripperMotorPin, OUTPUT);
  gripperMotor.attach(gripperMotorPin);
  pinMode(liftMotorPin, OUTPUT);
  liftMotor.attach(liftMotorPin);


 
 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////          VOID LOOP           ///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {
  //Emergency shutoff
  shutoffPressed = detectProgramShutoff();
  if(shutoffPressed == 1){
    Serial.println("Shut Down");
    RUN_PROGRAM = false;
    stop();
  }


  //Getting first button info
  if(!firstButtonPressed){
    shelfPosition1 = receiveRemoteData1();
    if(shelfPosition1 != 0){
      firstButtonPressed = true;
      Serial.println(shelfPosition1);
      if(shelfPosition1 == 1 || shelfPosition1 == 3 || shelfPosition1 == 5){
      branch1isOnTheLeft = 1;
      } else {
        branch1isOnTheLeft = 0;
      }
      if(shelfPosition1 == 2 || shelfPosition1 == 1){
        branchNumber1 = 1;
        branchesLeft1 = 2;
      }
      if(shelfPosition1 == 4 || shelfPosition1 == 3){
        branchNumber1 = 2;
        branchesLeft1 = 1;
      }
      if(shelfPosition1 == 5 || shelfPosition1 == 6){
        branchNumber1 = 3;
        branchesLeft1 = 0;
      }
    }
  }
  //Detecting enter pressed for first button
  else if(enterPressed1 == 0){
    enterPressed1 = detectEnterPressed1();
    Serial.println("please press enter 1:");
  }
  //Getting second button info
  else if(!secondButtonPressed){
    shelfPosition2 = receiveRemoteData2();
    Serial.println("enter 1 pressed:");
    if(shelfPosition2 != 0){
      secondButtonPressed = true;
      branchNumber2 = shelfPosition2;
      Serial.println(branchNumber2);
      RUN_PROGRAM = true; stage1 = true;
      //For testing purposes to start right before ending junction, comment out to run full code from beginning
      // stage1 = false;
      // stage6c = true;
    }
  }
  //Detecting enter pressed for second button
  // else if(enterPressed2 == 0){
  //   enterPressed2 = detectEnterPressed2();
  //   Serial.println("please press enter 2:");
  //}


  //Both shelf positions have been read in
  //Now start on the main program
  if (RUN_PROGRAM){
    Serial.println("Program running");
    rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
    leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
    EXTrightIRSensorValue = digitalRead(EXT_IR_SENSOR_RIGHT);
    EXTleftIRSensorValue = digitalRead(EXT_IR_SENSOR_LEFT);
    //Start stage1 which is line tracking until the right branch is reached
    if (stage1) {
      //Line follow
      lineDetect(rightIRSensorValue, leftIRSensorValue, stopTracking1);
      //If front IRs see a branch: pause
      if(rightIRSensorValue == HIGH && leftIRSensorValue == HIGH && BranchDetect1 == false){
        stop();
        // delay(500);
        branchCounter1++;
        if(branchCounter1 < branchNumber1){
          forward();
          delay(500);
        }
        else if (branchCounter1 == branchNumber1){
          // stopTracking1 = true;
          // BranchDetect1 = true;
          // lastBranch1 = true;
          stage1 = false; stage2 = true;
        }
      }
    }
    ////Go forward until external irs are on line and raise lift
    else if(stage2) {
      forwardSlow();
      if(EXTrightIRSensorValue == HIGH && EXTleftIRSensorValue == HIGH){
        stop();
        // delay(500);
        //raise lift
        motorControl(80, liftMotor);
        delay(3500);
        motorControl(0, liftMotor);
        stage2 = false; stage2b = true;
      }
    }
    // go forward until ext irs are off black
    else if(stage2b) {
      forwardSlow();
      if(EXTrightIRSensorValue == LOW || EXTleftIRSensorValue == LOW){
        stop();
        // delay(300);
        stage2b = false; stage2c = true;
      }
    }
    // turn right or left until opposite ir is on branch
    else if(stage2c){
      if(branch1isOnTheLeft == 1){
        turnLeft();
        if(switch4){
          delay(1500);
          switch4 = false;
        }
        else if(rightIRSensorValue == HIGH) {
          stop();
          // delay(300);
          stage2c = false; stage3 = true;
        }
      }
      else {
        turnRight();
        if(switch4){
          delay(1500);
          switch4 = false;
        }
        else if(leftIRSensorValue == HIGH) {
          stop();
          // delay(300);
          stage2c = false; stage3 = true;
        }
      }
        // stop();
        // delay(500);
    }
    //Going up to the wall and grabbing juice box
    else if(stage3) {
      distanceR = detectDistanceR();
      distanceL = detectDistanceL();
      lineDetectAndAvoidObstacles(rightIRSensorValue, leftIRSensorValue, false);
      if(distanceR <= stoppingDistance && distanceL <= stoppingDistance){
        stop();
        // delay(500);
        //Grab juice box and pickup slightly
        motorControl(-40, gripperMotor);
        delay(1700);
        motorControl(0, gripperMotor);
        motorControl(80, liftMotor);
        delay(300);
        motorControl(0, liftMotor);
        stage3 = false; stage4 = true;
      }
    }
    //Reverse back to central black line
    else if(stage4) {
      backwardSlow();
      if(EXTrightIRSensorValue == HIGH && EXTleftIRSensorValue == HIGH){
        stop();
        // delay(200);
        // lower lift
        motorControl(-80, liftMotor);
        delay(2200);
        motorControl(0, liftMotor);
        stage4 = false; stage4b = true;
      }
      //lower lift
     
      //   backwardSlow();
      //   delay(1800);
      //   stop();
      //   delay(200);
      //   //Lower lift
     
      //   delay(500);
      //   if(branch1isOnTheLeft == 1){
      //     turnRightLeftWheelsOnly();
      //     delay(3600);
      //     stop();
      //     delay(500);
      //   }
      //   else {
      //     turnLeftRightWheelsOnly();
      //     delay(3600);
      //     stop();
      //     delay(500);
         
      //   }
      //   if(branchesLeft1 == 0){
      //     stage4 = false; stage6 = true;
      //   } else {
      //     stage4 = false; stage5 = true;
      //   }
      // }
    }
    // go forward until ext irs are off line
    else if(stage4b) {
      forwardSlow();
      if(EXTrightIRSensorValue == LOW || EXTleftIRSensorValue == LOW){
        stop();
        // delay(300);
        stage4b = false; stage4c = true;
        stop();
        // delay(200);
        backwardSlow();
        delay(2000);
        stop();
        // delay(200);
      }
    }
    //turn correct way back on to line and stop when opposite front sensor sees line
    else if(stage4c) {
      if(branch1isOnTheLeft == 1){
        turnRightLeftWheelsOnly();
        if(switch5){
          delay(3500);
          switch5 = false;
        }
        if(leftIRSensorValue == HIGH) {
          stop();
          // delay(300);
          if(branchesLeft1 == 0){
            stage4c = false; stage6 = true;
          } else {
            stage4c = false; stage5 = true;
          }
        }
      }
      else {
        turnLeftRightWheelsOnly();
        if(switch5){
          delay(3500);
          switch5 = false;
        }
        else if(rightIRSensorValue == HIGH) {
          stop();
          // delay(300);
          if(branchesLeft1 == 0){
            stage4c = false; stage6 = true;
          } else {
            stage4c = false; stage5 = true;
          }
        }
      }
    }
    //line follow until end of starting junctions
    else if(stage5) {
      lineDetect(rightIRSensorValue, leftIRSensorValue, false);
      if(EXTrightIRSensorValue == HIGH && EXTleftIRSensorValue == HIGH) {
        stop();
        // delay(500);
        if(branchesLeft1 > 0) {
          forward();
          delay(500);
          branchesLeft1--;
        }
        if (branchesLeft1 == 0) {
          forward();
          delay(500);
          stage5 = false; stage6 = true;
        }
      }
    }
    //deal with moving obstacle
    else if(stage6){
      delayCounter++;
      if(delayCounter < 160){
        lineDetect(rightIRSensorValue, leftIRSensorValue, false);
      }else if (delayCounter == 160){
        stop();
        // delay(500);
        //raise lift
        motorControl(80, liftMotor);
        delay(2600);
        motorControl(0, liftMotor);
        motorControl(-40, gripperMotor);
        delay(500);
        motorControl(0, gripperMotor);
      }
      else if(delayCounter > 160){
        distanceR = detectDistanceR();
        distanceL = detectDistanceL();
        lineDetectAndAvoidObstacles(rightIRSensorValue, leftIRSensorValue, false);
        if(distanceR <= stoppingDistance3 || distanceL <= stoppingDistance3){
          stop();
          // delay(300);
          stage6 = false; stage6b = true;
        }
      }
    }
    //wait for obstacle to move, lower lift
    else if(stage6b) {
      distanceR = detectDistanceR();
      distanceL = detectDistanceL();
      if(distanceR > stoppingDistance3 + 1 && distanceL > stoppingDistance3 + 1)
      {
        delay(1000);
        motorControl(-80, liftMotor);
        delay(2100);
        motorControl(0, liftMotor);
        stage6b = false; stage6c = true;
      }
    }
    //line follow until start of ending junction
    else if(stage6c) {
      lineDetect2(rightIRSensorValue, leftIRSensorValue, false);
      if(EXTrightIRSensorValue == HIGH && EXTleftIRSensorValue == HIGH) {
        branchCounter2++;
        stop();
        // delay(500);
        if(branchNumber2 == 1){
          forwardSlow();
          delay(500);
          stop();
          // delay(300);
          stage6c = false; stage8 = true;
        } else {
          stop();
          // delay(200);
          backwardSlow();
          delay(2050);
          stop();
          // delay(200);
          stage6c = false; stage6e = true;
        }
      }
    }
    //drive forward until ext ir sensors are off black                             //skip this step
    else if(stage6d) {
      forwardSlow();
      if(EXTrightIRSensorValue == LOW || EXTleftIRSensorValue == LOW){
        stop();
        // delay(300);
        stage6d = false; stage6e = true;
      }
    }
    //Turn left until front right ir sensor is on line
    else if(stage6e) {
      turnLeftRightWheelsOnly();
      if(switch1){
        delay(3500);
        switch1 = false;
      }
      else if(rightIRSensorValue == HIGH) {
        stop();
        // delay(300);
        stage6e = false; stage7 = true;
      }
    }
    //line follow until correct junction and stop
    else if(stage7) {
      lineDetect2(rightIRSensorValue, leftIRSensorValue, false);
      if(EXTrightIRSensorValue == HIGH) {
        stop();
        // delay(500);
        branchCounter2++;
        if(branchNumber2 != branchCounter2){
          forward();
          delay(500);
        }
        else if(branchNumber2 == branchCounter2) {
          stop();
          // delay(200);
          backwardSlow();
          delay(2200);
          stop();
          // delay(200);
          stage7 = false; stage7c = true;
        }
      }
    }
    //Drive forward until ext right ir is off black                    //skip this step
    else if(stage7b) {
      forwardSlow();
      if(EXTrightIRSensorValue == LOW){
        stop();
        // delay(300);
        stage7b = false; stage7c = true;
      }
    }
    // turn right until front left ir sensor is on branch
    else if(stage7c) {
      turnRightLeftWheelsOnly();
      // if(switch2){
      delay(3600);
      stop();
      stage7c = false; stage8 = true;
      // switch2 = false;
      // }
      // else if(leftIRSensorValue == HIGH) {
      //   stop();
      //   delay(300);
      //   stage7c = false; stage8 = true;
      // }
    }
    //drop off juice box
    else if(stage8) {
      //delayCounter2++;
      //lineDetect(rightIRSensorValue, leftIRSensorValue, false);
      //if (delayCounter2 > 60){
        //stop();
        //delay(300);
        motorControl(-80, liftMotor);
        delay(200);
        motorControl(0, liftMotor);
        motorControl(70, gripperMotor);
        delay(1200);
        motorControl(0,gripperMotor);
        delay(300);
        stage8 = false; stage9 = true;
      //}
    }
    //Reverse back to main line
    else if(stage9) {
      backwardSlow();
      if(EXTrightIRSensorValue == HIGH || EXTleftIRSensorValue == HIGH){
        stop();
        // motorControl(80, liftMotor);
        // delay(3000);
        // motorControl(0, liftMotor);
        // delay(300);
        // turnLeft();
        // delay(1500);
        stage9 = false; stage9b = true;
      }
    }
    //forward until ext ir sensors are off
    else if(stage9b) {
      forwardSlow();
      if(EXTrightIRSensorValue == LOW || EXTleftIRSensorValue == LOW){
        stop();
        // delay(300);
        stage9b = false; stage9c = true;
      }
    }
    //turn left until right ir sensor is on main line
    else if(stage9c) {
      turnLeft();
      if(switch3){
        delay(1000);
        switch3 = false;
      }
      else if(rightIRSensorValue == HIGH) {
        stop();
        // delay(300);
        stage9c = false; stage10 = true;
      }
    }
    // go until last wall is reached
    else if(stage10) {
      lineDetect2(rightIRSensorValue, leftIRSensorValue, false);
      distanceR = detectDistanceR();
      distanceL = detectDistanceL();
      if(distanceR <= stoppingDistance2 && distanceL <= stoppingDistance2){
        stop();
        stage10 = false; stage11 = true;
      }
    }
    else if(stage11) {
      stop();
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////          FUNCTION DEFINITIONS           ////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void lineDetect(int rightIRSensorValue, int leftIRSensorValue, bool stopTracking1){
 
  if(stopTracking1 == false){
      //If none of the sensors detects black line, then go straight
      if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
      {
        forward();
      }
      //If right sensor detects black line, then turn right
      else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
      {
        turnRightLine();
      }
      //If left sensor detects black line, then turn left  
      else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
      {
          turnLeftLine();
      }
      //If both the sensors detect black line, then stop tracking      
      else
      {
        forward();
      }
  }
}

void lineDetect2(int rightIRSensorValue, int leftIRSensorValue, bool stopTracking1){
 
  if(stopTracking1 == false){
      //If none of the sensors detects black line, then go straight
      if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
      {
        forward2();
      }
      //If right sensor detects black line, then turn right
      else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
      {
        turnRightLine();
      }
      //If left sensor detects black line, then turn left  
      else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
      {
        turnLeftLine();
      }
      //If both the sensors detect black line, then stop tracking      
      else
      {
        forward2();
      }
  }
}


void lineDetectAndAvoidObstacles(int rightIRSensorValue, int leftIRSensorValue, bool stopTracking2){
  if(stopTracking2 == false){
    //If none of the sensors detects black line, then go straight
      if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
      {
        forwardSlow();
      }
      //If right sensor detects black line, then turn right
      else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
      {
          turnRightLine();
      }
      //If left sensor detects black line, then turn left  
      else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
      {
          turnLeftLine();
      }
      //If both the sensors detect black line, then stop tracking      
      else
      {
        //nothing
      }
   }
}


float detectDistanceR(){
 
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);


  durationR = pulseIn(echoPinR, HIGH);
  distanceR = (durationR*.0343)/2;
 
  Serial.print("Distance: ");
  Serial.println(distanceR);
  delay(0.05);


  return distanceR;
}


float detectDistanceL(){
 
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);


  durationL = pulseIn(echoPinL, HIGH);
  distanceL = (durationL*.0343)/2;
 
  Serial.print("Distance: ");
  Serial.println(distanceL);
  delay(0.05);


  return distanceL;
}


int receiveRemoteData1(){
  //if(firstButtonPressed == false){
  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button);
  switch(IRrecv_button)
      {
        case 6:
            //firstButtonPressed = true;
            branchNumber1 = 1;
            return branchNumber1;
          break;
        case 7:
            //firstButtonPressed = true;
            branchNumber1 = 2;
            return branchNumber1;
          break;
        case 8:
            //firstButtonPressed = true;
            branchNumber1 = 3;
            return branchNumber1;
          break;  
        case 9:
            //firstButtonPressed = true;
            branchNumber1 = 4;
            return branchNumber1;
          break;
        case 10:
            //firstButtonPressed = true;
            branchNumber1 = 5;
            return branchNumber1;
          break;
        case 11:
            //firstButtonPressed = true;
            branchNumber1 = 6;
            return branchNumber1;
          break;
        default:
          return 0;
          break;
      }
  //}
}


int receiveRemoteData2(){
  //if(secondButtonPressed == false){
  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button);
  switch(IRrecv_button)
      {
        case 6:
            //secondButtonPressed = true;
            branchNumber2 = 1;
            return branchNumber2;
          break;
        case 7:
            //secondButtonPressed = true;
            branchNumber2 = 2;
            return branchNumber2;
          break;
        case 8:
            //secondButtonPressed = true;
            branchNumber2 = 3;
            return branchNumber2;
          break;  
        case 9:
            //secondButtonPressed = true;
            branchNumber2 = 4;
            return branchNumber2;
          break;
        case 10:
            //secondButtonPressed = true;
            branchNumber2 = 5;
            return branchNumber2;
          break;
        case 11:
            //secondButtonPressed = true;
            branchNumber2 = 6;
            return branchNumber2;
          break;
        default:
          return 0;
          break;
        }
  //}
}


int detectEnterPressed1(){
 
  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button);
  switch(IRrecv_button)
      {
        case 5:
            //secondButtonPressed = true;
            enterPressed1 = 1;
            return enterPressed1;
          break;
        default:
          return 0;
          break;
        }
}


int detectProgramShutoff(){
 
  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button);
  switch(IRrecv_button)
      {
        case 1:
            //secondButtonPressed = true;
            //enterPressed1 = 1;
            return 1;
          break;
        default:
          return 0;
          break;
        }
}


int detectEnterPressed2(){
 
  IRremote.DeviceDriverSet_IRrecv_Get(&IRrecv_button);
  switch(IRrecv_button)
      {
        case 5:
            //secondButtonPressed = true;
            enterPressed2 = 1;
            return enterPressed2;
          break;
        default:
          return 0;
          break;
        }
}


void forward(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed1);  
    analogWrite(rightWheels, speed1);
    digitalWrite(leftWheelsDir, HIGH);
    digitalWrite(rightWheelsDir, HIGH);
   
}

void forward2(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed5);  
    analogWrite(rightWheels, speed5);
    digitalWrite(leftWheelsDir, HIGH);
    digitalWrite(rightWheelsDir, HIGH);
   
}


void forwardSlow(){
  digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed2);  
    analogWrite(rightWheels, speed2);
    digitalWrite(leftWheelsDir, HIGH);
    digitalWrite(rightWheelsDir, HIGH);
}


void turnLeft(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed4);  
    analogWrite(rightWheels, speed4);
    digitalWrite(leftWheelsDir, LOW);
    digitalWrite(rightWheelsDir, HIGH);
}


void turnLeftLine(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed4 - 20);  
    analogWrite(rightWheels, speed4);
    digitalWrite(leftWheelsDir, LOW);
    digitalWrite(rightWheelsDir, HIGH);
}


void turnLeftRightWheelsOnly(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed2);  
    analogWrite(rightWheels, speed4);
    digitalWrite(leftWheelsDir, LOW);
    digitalWrite(rightWheelsDir, HIGH);
}


void turnLeft2(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, 100);  
    analogWrite(rightWheels, 15);
    digitalWrite(leftWheelsDir, LOW);
    digitalWrite(rightWheelsDir, LOW);
}


void turnRight(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed4);  
    analogWrite(rightWheels, speed4);
    digitalWrite(leftWheelsDir, HIGH);
    digitalWrite(rightWheelsDir, LOW);
}


void turnRightLine(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed4);  
    analogWrite(rightWheels, speed4 - 20);
    digitalWrite(leftWheelsDir, HIGH);
    digitalWrite(rightWheelsDir, LOW);
}


void turnRightLeftWheelsOnly(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed4);  
    analogWrite(rightWheels, speed2);
    digitalWrite(leftWheelsDir, HIGH);
    digitalWrite(rightWheelsDir, LOW);
}


void turnRight2(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, 15);  
    analogWrite(rightWheels, 100 );
    digitalWrite(leftWheelsDir, LOW);
    digitalWrite(rightWheelsDir, LOW);
}


void backward(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed1);  
    analogWrite(rightWheels, speed1);
    digitalWrite(leftWheelsDir, LOW);
    digitalWrite(rightWheelsDir, LOW);
}


void backwardSlow(){
    digitalWrite(motorsEnable, HIGH);
    analogWrite(leftWheels, speed2);  
    analogWrite(rightWheels, speed2);
    digitalWrite(leftWheelsDir, LOW);
    digitalWrite(rightWheelsDir, LOW);
}


void stop(){
    digitalWrite(motorsEnable, LOW);
    analogWrite(leftWheels, 0);  
    analogWrite(rightWheels, 0);
    digitalWrite(leftWheelsDir, LOW);
    digitalWrite(rightWheelsDir, LOW);  
}


int motorControl(int value, Servo motor){ // Gripper: + open, - close; Lift: + up, - down
  motor.write(map(value,-100,100,1150,1850));
}



