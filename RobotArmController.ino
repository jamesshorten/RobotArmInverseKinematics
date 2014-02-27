#include <DynamixelSerial3.h>
#include <cmath>
#include <cstdlib>
#include "IKTypes.h"

#define L_1 25.5
#define L_2 26.0
#define L_SPOON 17.117
#define ANGLE_SPOON 0.1171
#
#define ELBOW_OFFSET_ANGLE 23.0
#define WRIST_OFFSET_ANGLE 22.0
#define _USE_MATH_DEFINES

Position currentPos;

bool getJointAnglesInDegrees(double x, double y, double z, double handAngle,

 double &shoulderJointAngleRotation,
 double &shoulderJointAngleElevation, double &elbowJointAngle,
 double &wristJointAngle) {
  if (sqrt(x*x + y*y + z*z) > L_1 + L_2)
    return false;

  if (y == 0.0) y = 0.00001;
  if (x == 0.0) x = 0.00001;

  handAngle *= 0.0174532925;
  double theta  = fabs(handAngle) + ANGLE_SPOON;

  double deltaZ = L_SPOON * sin(theta);
  double deltaR = L_SPOON * cos(theta);

  double r = sqrt(x * x + y * y);

  r -= deltaR;
  z += deltaZ;
  shoulderJointAngleRotation = atan2(y, x);
  double b = sqrt(r * r + z * z);
  double q1 = atan2(z, r);
  double q2 = acos((L_1 * L_1 - L_2 * L_2 + b * b) / (2.0 * L_1 * b));
  shoulderJointAngleElevation = q1 + q2;
  elbowJointAngle = acos((L_1 * L_1 + L_2 * L_2 - b * b) / (2.0 * L_1 * L_2));
  wristJointAngle = handAngle - elbowJointAngle - shoulderJointAngleElevation;
  if (wristJointAngle < 0) 
    wristJointAngle += 2*M_PI;

  wristJointAngle *= 180.0/M_PI; 
  elbowJointAngle *= 180/M_PI;
  shoulderJointAngleElevation *= 180/M_PI;
  shoulderJointAngleRotation *= 180/M_PI;

  wristJointAngle = -WRIST_OFFSET_ANGLE + wristJointAngle + 20;
  elbowJointAngle += ELBOW_OFFSET_ANGLE;
  shoulderJointAngleElevation = 270 - shoulderJointAngleElevation;   
  return true;
}

bool moveToPositionInTime(Position newPosition, Position currentPosition, double time) {
  Pose pose = {0,0,0,0};
  Pose oldPose = {0.0, 0.0, 0.0, 0.0};

  if(!getJointAnglesInDegrees(newPosition.x, newPosition.y, newPosition.z, newPosition.angle, pose.a1, pose.a2, pose.a3, pose.a4) ||
    !getJointAnglesInDegrees(currentPosition.x, currentPosition.y, currentPosition.z, currentPosition.angle, oldPose.a1, oldPose.a2, oldPose.a3, oldPose.a4))
    return false;

  int moveSpeed1 = (int) round((fabs(pose.a1 - oldPose.a1)/time)/0.666);
  int moveSpeed2 = (int) round((fabs(pose.a2 - oldPose.a2)/time)/0.684);
  int moveSpeed3 = (int) round((fabs(pose.a3 - oldPose.a3)/time)/0.684);
  int moveSpeed4 = (int) round((fabs(pose.a4 - oldPose.a4)/time)/0.666);

  Dynamixel.moveSpeed(2, (int) pose.a1/0.29, moveSpeed1);
  Dynamixel.moveSpeed(3, (int) pose.a2/0.088, moveSpeed2);
  Dynamixel.moveSpeed(4, (int) pose.a3/0.088, moveSpeed3);
  Dynamixel.moveSpeed(5, (int) pose.a4/0.29, moveSpeed4);

  return true;
}

bool moveToPositionWithSpeed(Position position, int moveSpeed) {
  Pose pose = {0,0,0,0};

  if(!getJointAnglesInDegrees(position.x, position.y, position.z, position.angle, pose.a1, pose.a2, pose.a3, pose.a4))
    return false;

  Dynamixel.moveSpeed(2, (int) pose.a1/0.29, moveSpeed*2);
  Dynamixel.moveSpeed(3, (int) pose.a2/0.088, moveSpeed);
  Dynamixel.moveSpeed(4, (int) pose.a3/0.088, moveSpeed);
  Dynamixel.moveSpeed(5, (int) pose.a4/0.29, moveSpeed*2);

  return true;
}


void unrecognizedCmd (const char *command) {
  Serial.print("Unrecognized Command: \"");
  Serial.print(command);
  Serial.println("\"");
}

bool moveAtMetersPerSecond(Position pos, Position currentPosition, double spd) {
  double distance = sqrt(pow(pos.x - currentPosition.x, 2) + pow(pos.y - currentPosition.y, 2) + pow(pos.z - currentPosition.z, 2));
  double time = distance/spd/10.0;
  return moveToPositionInTime(pos, currentPosition, time);
}

bool straightLineMove(Position pos, Position currentPos, double speed) {
  Position deltaPos = {pos.x - currentPos.x, pos.y - currentPos.y, pos.z - currentPos.z, pos.angle - currentPos.angle};
  double length = sqrt(pow(deltaPos.x, 2) + pow(deltaPos.y, 2) + pow(deltaPos.y, 2));
  int steps = (int) ceil(length/5);
  Position posIncrement = {deltaPos.x/steps, deltaPos.y/5, deltaPos.z/5, deltaPos.angle/5};

  Pose* poseArray  = new Pose[steps];
  Position* positionArray = new Position[steps];

  for(int i = 0; i < steps; i++) {
    positionArray[i] = {currentPos.x + posIncrement.x*(i+1), currentPos.y + posIncrement.y*(i+1), currentPos.z + posIncrement.z*(i+1) + currentPos.angle + posIncrement.angle*(i+1)};
    poseArray[i] = {0.0, 0.0, 0.0, 0.0};
    getJointAnglesInDegrees(positionArray[i].x, positionArray[i].y, positionArray[i].z, positionArray[i].angle, poseArray[i].a1, poseArray[i].a2, poseArray[i].a3, poseArray[i].a4);
  }

  double stepTime = 5/speed/10;  
  Pose lastPose = {0.0, 0.0, 0.0, 0.0};
  getJointAnglesInDegrees(currentPos.x, currentPos.y, currentPos.z, currentPos.angle, lastPose.a1, lastPose.a2, lastPose.a3, lastPose.a4);

  for(int i = 0; i < steps; i++) {
    int moveSpeed1 = (int) round((fabs(poseArray[i].a1 - lastPose.a1)/stepTime)/0.666);
    int moveSpeed2 = (int) round((fabs(poseArray[i].a2 - lastPose.a2)/stepTime)/0.684);
    int moveSpeed3 = (int) round((fabs(poseArray[i].a3 - lastPose.a3)/stepTime)/0.684);
    int moveSpeed4 = (int) round((fabs(poseArray[i].a4 - lastPose.a4)/stepTime)/0.666);
    lastPose = poseArray[i];

    Dynamixel.moveSpeed(2, (int) poseArray[i].a1/0.29, moveSpeed1);
    Dynamixel.moveSpeed(3, (int) poseArray[i].a2/0.088, moveSpeed2);
    Dynamixel.moveSpeed(4, (int) poseArray[i].a3/0.088, moveSpeed3);
    Dynamixel.moveSpeed(5, (int) poseArray[i].a4/0.29, moveSpeed4);

    delay((int) stepTime*0.8*1000);
  }


  delete[] poseArray;
  delete[] positionArray;
  
  return true;
}

void scoop(double x, double y, double z, double angle, double speed) {
  double time = 1.5;

  Position p1, p2, p2a, p3, p4;
  p1 = {-20.0, 0.0, 8.0, -70.0};
  p2 = {-22.0, 0.0, -5.0, -70.0};
  p2a = {-32.0, 0.0, 5.0, 0};
  p3 = {-30.0, 0.0, 15, 0.0};
  p4 = {x, y, z, angle};

  moveAtMetersPerSecond(p1, currentPos, speed);
  currentPos = p1;
  delay((int) (2*time + 0.5)*1000);
  moveToPositionInTime(p2, currentPos, time);
  currentPos = p2;
   delay((int) (time + 0.5)*1000);
  moveToPositionInTime(p2a, currentPos, time);
  currentPos = p2a;
  delay((int) (time + 0.5)*1000);
  moveToPositionInTime(p3, currentPos, time);
  currentPos = p3;
  delay((int) (time + 0.5)*1000);
  moveAtMetersPerSecond(p4, currentPos, speed);
  currentPos = p4;


}

void processCommands() {
  char buffer[100]; // Buffer to hold the text from the serial port
  int bytesRead = Serial.readBytesUntil(' ', buffer, 99); // Read a command char from the serial port;
  if(bytesRead == 1) {
    switch(buffer[0]) {
      case 'A': {
        int bytesRead = Serial.readBytesUntil('\n', buffer, 99);
        Position AbortPos = {0.0, 25.0, 7.0, 0.0};
        moveAtMetersPerSecond(AbortPos, currentPos,  1.0);
        currentPos = AbortPos;
        break;
      }
      case 'S': {
        double x = Serial.parseFloat();
        double y = Serial.parseFloat();
        double z = Serial.parseFloat();
        double angle = Serial.parseFloat();
        double speed = Serial.parseFloat();

        if(speed != 0.0) {
          scoop(x, y, z, angle, speed);

        } else {
          Serial.println("Usage: \"S x y z angle speed\"\n All paramters are floats and speed > 0");
        }
        
        break;
      }
      case 'M': {
        double x = Serial.parseFloat();
        double y = Serial.parseFloat();
        double z = Serial.parseFloat();
        double angle = Serial.parseFloat();
        double speed = Serial.parseFloat();

        if(speed != 0.0) {
          Position pos = {x, y, z, angle};
          if(!moveAtMetersPerSecond(pos, currentPos, speed))
            Serial.println("Invalid position");
          else currentPos = pos;

        } else {
          Serial.println("Usage: \"M x y z angle speed\"\n All paramters are floats and speed > 0");
        }
        break;
      }
      default: {
        Serial.println("Command Not recognized");
        int bytesRead = Serial.readBytesUntil('\n', buffer, 99);
        break;
      }
    }

  } else if (bytesRead > 1) {
    Serial.println("Commands are only 1 char long");
  }
}

void setup(){
  Serial.begin(115200);
  Dynamixel.begin(57142,2);  // Initialize the servo at 1Mbps and Pin Control 2  

  delay(1000);
  Dynamixel.setCSlope(2, 128, 128);
  Dynamixel.setCSlope(5, 64, 64);
  
  Dynamixel.setPID(3, 32, 20, 20);
  Dynamixel.setPID(4, 32, 20, 20);

  delay (500);
  
  Position pos = {0.0, 25.0, 7.0, 0.0};
  moveToPositionWithSpeed(pos, 50);
  currentPos = pos;
  delay(5000);
}

void loop(){
  processCommands();
  //Serial.println("Ready");
  delay(1000);
}