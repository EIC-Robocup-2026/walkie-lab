#include <Arduino.h>
#include <Servo.h>

// lengths of the robotic arm segments in meters
double l1 = 0.105; // length of link 1 controled by servo 2
double l2 = 0.125; // length of link 2 controled by servo 3
double l3 = 0.175; // length of link 3 controled by servo 4

Servo servo_1; // servo for base rotation
Servo servo_2; // servo for first link joint
Servo servo_3; // servo for second link joint
Servo servo_4; // servo for third link joint
Servo servo_5; // servo for end effector rotation
Servo servo_6; // servo for end effector gripper

Servo servos[6] = {servo_1, servo_2, servo_3, servo_4, servo_5, servo_6};

// when initializing the servos the arm should be in the upright position with open gripper
double initializingTheta_1 = 90;
double initializingTheta_2 = 90;
double initializingTheta_3 = 90;
double initializingTheta_4 = 90;
double initializingTheta_5 = 0;
double initializingTheta_6 = 90;

double initializingThetas[6] = {
    initializingTheta_1,
    initializingTheta_2,
    initializingTheta_3,
    initializingTheta_4,
    initializingTheta_5,
    initializingTheta_6};

void setup()
{

  // Lolin ESP8266 NodeMCU V3 pinout
  servo_1.attach(5, 500, 2500);  // D1
  servo_2.attach(4, 500, 2500);  // D2
  servo_3.attach(0, 500, 2500);  // D3
  servo_4.attach(14, 500, 2500); // D5
  servo_5.attach(12, 500, 2500); // D6
  servo_6.attach(13, 500, 2500); // D7

  Serial.begin(9600);

  for (int i = 0; i < 6; i++)
  {
    servos[i].write(initializingThetas[i]);
  }

  delay(3000); // delayed for servos to reach initializing positions
  Serial.println("Robotic arm initialized with servos at initial angles.");
}

void getUnitVector3D(double x, double y, double z, double *unitVector)
{
  double magnitude = sqrt(x * x + y * y + z * z);
  if (magnitude == 0)
  {
    unitVector[0] = unitVector[1] = unitVector[2] = 0;
    return;
  }
  unitVector[0] = x / magnitude;
  unitVector[1] = y / magnitude;
  unitVector[2] = z / magnitude;
}

double getMagnitude2D(double x1, double y1, double x2, double y2){
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double getDotProduct2D(double x1, double y1, double x2, double y2){
  return (x1 * x2 + y1 * y2);
}

double angleBetweenVectors2D(double x1, double y1, double x2, double y2){
  double dotProduct = getDotProduct2D(x1, y1, x2, y2);
  double magnitude1 = getMagnitude2D(0, 0, x1, y1);
  double magnitude2 = getMagnitude2D(0, 0, x2, y2);
  if (magnitude1 == 0 || magnitude2 == 0) return 0; // avoid division by zero 
  return acos(dotProduct / (magnitude1 * magnitude2));
}

//check if angle on vector 2 is anti-clockwise compared to vector 1
// returns true if the angle is anti-clockwise, false if clockwise
bool isAntiClockwise(double x1, double y1, double x2, double y2){
  return (x1 * -y2 + y1 * x2) > 0; // returns true if the angle is anti-clockwise
}

void loop(){
  if (Serial.available()) {
      String data = Serial.readStringUntil('\n'); // Read until newline
      data.trim(); // Remove spaces or \r

      // Find comma positions
      int c1 = data.indexOf(',');
      int c2 = data.indexOf(',', c1 + 1);
      int c3 = data.indexOf(',', c2 + 1);

      double dx;
      double dy;
      double dz;
      double roll;

      if (c1 > 0 && c2 > c1 && c3 > c2) {
        dx = data.substring(0, c1).toDouble();
        dy = data.substring(c1 + 1, c2).toDouble();
        dz = data.substring(c2 + 1, c3).toDouble();
        roll = data.substring(c3 + 1).toDouble();
      }
      Serial.println("input recieved");
      Serial.println(dx);
      Serial.println(dy);
      Serial.println(dz);
      Serial.println(roll);
      


      double gripper_intesity; 
      bool mission_completed = false;
      //6 DOF servo arm cannot rotate in pitch and yaw



      // unit direction vector for the differential movement
      double differentialUnitDirection[3];
      getUnitVector3D(dx, dy, dz, differentialUnitDirection);

      // getting all the current angles of the servos
      double currentServoTheta[6];
      for (int i = 0; i < 6; i++)
      {
        currentServoTheta[i] = servos[i].read();
      }

      // new angles for the servos after the differential movement, im so fucking sorry idk how to make this shit cleaner
      double newTheta[6];

      // logic for rotating the base using dy as the input
      double kp = 1;                                                          // proportional gain for the control loop
      newTheta[0] = currentServoTheta[0] + differentialUnitDirection[1] * kp; // rotate the base

      // absolute angles of the joints for controlling in the z-x plane, based on servo 2,3,4 angles all CCD algorithm will run on 2D z-x plane
      double absoluteJointAngle[3];
      absoluteJointAngle[0] = currentServoTheta[1];
      absoluteJointAngle[1] = 90 - (180 - abs(currentServoTheta[1])) + absoluteJointAngle[0]; // joint 2 angle
      absoluteJointAngle[2] = 90 - (180 - abs(currentServoTheta[2])) + absoluteJointAngle[1]; // joint 3 angle

      // calculate all location of the joint, end effector position and target position on x-z plane
      double jointStartPoint_1[2] = {0, 0};
      double jointStartPoint_2[2] = {l1 * cos(radians(absoluteJointAngle[0])), l1 * sin(radians(absoluteJointAngle[0]))};
      double jointStartPoint_3[2] = {jointStartPoint_2[0] + l2 * cos(radians(absoluteJointAngle[1])), jointStartPoint_2[1] + l2 * sin(radians(absoluteJointAngle[1]))};
      double endEffectorLocation[2] = {jointStartPoint_3[0] + l3 * cos(radians(absoluteJointAngle[2])), jointStartPoint_3[1] + l3 * sin(radians(absoluteJointAngle[2]))};
      
      double targetLocation[2] = {endEffectorLocation[0] + differentialUnitDirection[0] * kp, endEffectorLocation[1] + differentialUnitDirection[2] * kp};

      // calculate angle between joint and target location — non directional
      double JointToTargetAngle_3 = angleBetweenVectors2D(jointStartPoint_3[0], jointStartPoint_3[1], targetLocation[0], targetLocation[1]);
      double JointToTargetAngle_2 = angleBetweenVectors2D(jointStartPoint_2[0], jointStartPoint_2[1], targetLocation[0], targetLocation[1]);
      double JointToTargetAngle_1 = angleBetweenVectors2D(jointStartPoint_1[0], jointStartPoint_1[1], targetLocation[0], targetLocation[1]);

      // determine if the angle is anti-clockwise or clockwise
      bool isAntiClockwise_3 = isAntiClockwise(jointStartPoint_3[0], jointStartPoint_3[1], targetLocation[0], targetLocation[1]);
      bool isAntiClockwise_2 = isAntiClockwise(jointStartPoint_2[0], jointStartPoint_2[1], targetLocation[0], targetLocation[1]);
      bool isAntiClockwise_1 = isAntiClockwise(jointStartPoint_1[0], jointStartPoint_1[1], targetLocation[0], targetLocation[1]);

      //assigning weight for CCD joint rotation
      double weight[3] = {0.3, 0.5, 0.8};
      (isAntiClockwise_3) ? newTheta[3] = currentServoTheta[3] + JointToTargetAngle_3 * weight[2] : newTheta[3] = currentServoTheta[3] - JointToTargetAngle_3 * weight[2]; // joint 4 angle
      (isAntiClockwise_2) ? newTheta[2] = currentServoTheta[2] + JointToTargetAngle_2 * weight[1] : newTheta[2] = currentServoTheta[2] - JointToTargetAngle_2 * weight[1];  // joint 3 angle
      (isAntiClockwise_1) ? newTheta[1] = currentServoTheta[1] + JointToTargetAngle_1 * weight[0] : newTheta[1] = currentServoTheta[1] - JointToTargetAngle_1 * weight[0];  // joint 2 angle

      
      newTheta[4] = currentServoTheta[4] + roll;
      newTheta[5] = 180* gripper_intesity;
      
      servo_1.write(newTheta[0]);
      servo_2.write(newTheta[1]);
      servo_3.write(newTheta[2]);
      servo_4.write(newTheta[3]);
      servo_5.write(newTheta[4]);
      servo_6.write(newTheta[5]);





      for(int i =0; i< 6;i++){
        
        Serial.println(newTheta[i]);
      }
    }
  
}
