#ifndef quad_omni_H
#define quad_omni_H
#include "actiondrv.h"
#include "mbed.h"
#include <cmath>
class quad_omni {
public:
  quad_omni(int, int, int, int, CAN*);
  void motorInitialization();
  void motorInitialization(int, int);
  void motorUpdate();
  void setAccelaration(int);
  void setDecelaration(int);
  void setVelocityX(int);
  void setVelocityY(int);
  void setMovementOption(int);
  void setMotor1Speed(int);
  void setMotor2Speed(int);
  void setMotor3Speed(int);
  void setMotor4Speed(int);
  void setRadiusW(float);
  void setRadiusR(float);
  void setTheta(float);
  void setVelocityA(float);
  int getMotor1Speed();
  int getMotor2Speed();
  int getMotor3Speed();
  int getMotor4Speed();


private:
   Mutex quad_omni_mutex;
  float PI = 3.14159265358979323846;
  actionDrv *motor1;
  actionDrv *motor2;
  actionDrv *motor3;
  actionDrv *motor4;
  int motor1Speed = 1000;
  int motor2Speed = 1000;
  int motor3Speed = 1000;
  int motor4Speed = 1000;
  int motor1stopdir = 1;
  int motor2stopdir = 1;
  int motor3stopdir = 1;
  int motor4stopdir = 1;
  // variable for the formula calaulation
  float radius_w = 735.0;  // radius of the wheel
  float radius_r = 3600.0; // radius of the robot from the wheel to the center
  float theta = PI/2; // current angle of the robot
  float vel_a = 80;        // angular velocity needed of the turning
  float vel_a_error = 0;
  int vel_x = 1000;      // Vx
  int vel_y = 1000;      // Vy
  int direction = 0;       // movement direction (turning)
  int option = 0; // 0 -- line movement; 1 -- turn left; 2 -- turn right

  // acceleration and deceleration
  int acc = 1000000;
  int dec = 1000000;
};

#endif
