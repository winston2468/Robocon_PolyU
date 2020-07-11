#include "quad_omni.h"

quad_omni::quad_omni(int _id1, int _id2, int _id3, int _id4, CAN* _can1)
{
    this->motor1 = new actionDrv(_id1,_can1);
    this->motor2 = new actionDrv(_id2,_can1);
    this->motor3 = new actionDrv(_id3,_can1);
    this->motor4 = new actionDrv(_id4,_can1);
       
}

//Initialization HF motor with default acc dec
void quad_omni::motorInitialization()
{
    //refer to manual for more detail 
    this->motor1->Enable();
    ThisThread::sleep_for(100);
    this->motor1->SetOperationalMode();
    ThisThread::sleep_for(100); 
    this->motor1->Configvelocity(acc, dec);
    ThisThread::sleep_for(100);
    
    //set speed to 0
    this->motor1->SetVelocity(0);
    ThisThread::sleep_for(100);

        //refer to manual for more detail 
    this->motor2->Enable();
    ThisThread::sleep_for(100);
    this->motor2->SetOperationalMode();
    ThisThread::sleep_for(100); 
    this->motor2->Configvelocity(acc, dec);
    ThisThread::sleep_for(100);
    
    //set speed to 0
    this->motor2->SetVelocity(0);
    ThisThread::sleep_for(100);
            //refer to manual for more detail 
    this->motor3->Enable();
    ThisThread::sleep_for(100);
    this->motor3->SetOperationalMode();
    ThisThread::sleep_for(100); 
    this->motor3->Configvelocity(acc, dec);
    ThisThread::sleep_for(100);
    
    //set speed to 0
    this->motor3->SetVelocity(0);
    ThisThread::sleep_for(100);
    
            //refer to manual for more detail 
    this->motor4->Enable();
    ThisThread::sleep_for(100);
    this->motor4->SetOperationalMode();
    ThisThread::sleep_for(100); 
    this->motor4->Configvelocity(acc, dec);
    ThisThread::sleep_for(100);
    
    //set speed to 0
    this->motor4->SetVelocity(0);
    ThisThread::sleep_for(100);

    


    
}

void quad_omni::motorInitialization(int _acc, int _dec)
{

    //refer to manual for more detail 
    this->motor1->Enable();
    ThisThread::sleep_for(100);
    this->motor1->SetOperationalMode();
    ThisThread::sleep_for(100); 
    this->motor1->Configvelocity(acc, dec);
    ThisThread::sleep_for(100);
    this->motor1->SetVelocity(0);
    ThisThread::sleep_for(100);

    //set speed to 0

    this->motor1->SetVelocity(0);
    ThisThread::sleep_for(100);

        //refer to manual for more detail 
    this->motor2->Enable();
    ThisThread::sleep_for(100);
    this->motor2->SetOperationalMode();
    ThisThread::sleep_for(100); 
    this->motor2->Configvelocity(acc, dec);
    ThisThread::sleep_for(100);
    
    //set speed to 0

    this->motor2->SetVelocity(0);
    ThisThread::sleep_for(100);
            //refer to manual for more detail 
    this->motor3->Enable();
    ThisThread::sleep_for(100);
    this->motor3->SetOperationalMode();
    ThisThread::sleep_for(100); 
    this->motor3->Configvelocity(acc, dec);
    ThisThread::sleep_for(100);
    
    //set speed to 0

    this->motor3->SetVelocity(0);
    ThisThread::sleep_for(100);
    
            //refer to manual for more detail 
    this->motor4->Enable();
    ThisThread::sleep_for(100);
    this->motor4->SetOperationalMode();
    ThisThread::sleep_for(100); 
    this->motor4->Configvelocity(acc, dec);
    ThisThread::sleep_for(100);
    
    //set speed to 0

    this->motor4->SetVelocity(0);
    ThisThread::sleep_for(100);

    


    
}



//function that will be run repeatedly
//i.e constanly update velocity
//motor1 use id 1, refer to the action 
void quad_omni::motorUpdate()
{
     
        // line movement; if the angle does not equal to the original one, turn it back when moving
        if(option == 0)
        {
            direction = 0;
        }
        // turn left
        else if(option == 1)
        {
            direction = 1;
            vel_a = 80;
        }
        // turn right
        else if(option == 2)
        {
            direction = -1;
            vel_a = 80;
        }
        else if(option == 3)
        {
            direction = 1;
            vel_a = 15;
        }
        else if(option == 4)
        {
            direction = -1;
            vel_a = 15;
        }
        else if(option == 5)
        {
            direction = 1;
            vel_a = 50;
        }
        else if(option == 6)
        {
            direction = -1;
            vel_a = 50;
        }

        motor1Speed = ceil(((float)vel_y * cos(theta + (PI / 4)) - (float)vel_x * sin(theta + (PI / 4)) + direction * vel_a * radius_r + vel_a_error * radius_r) / radius_w);
        motor2Speed = ceil(((float)vel_y * cos(theta + (3 * PI / 4)) - (float)vel_x * sin(theta + (3 * PI / 4)) + direction * vel_a * radius_r + vel_a_error * radius_r) / radius_w);
        motor3Speed = ceil(((float)vel_y * cos(theta + (5 * PI / 4)) - (float)vel_x * sin(theta + (5 * PI / 4)) + direction * vel_a * radius_r + vel_a_error * radius_r) / radius_w);
        motor4Speed = ceil(((float)vel_y * cos(theta + (7 * PI / 4)) - (float)vel_x * sin(theta + (7 * PI / 4)) + direction * vel_a * radius_r + vel_a_error * radius_r) / radius_w);

        /*
        if(motor1Speed<0) {
            motor1Speed-=1;
        }
        if(motor2Speed<0) {
            motor2Speed-=1;
        }
        if(motor3Speed<0) {
            motor3Speed-=1;
        }
        if(motor4Speed<0) {
            motor4Speed-=1;
        }
        if(motor1Speed==1) {
            motor1Speed=0;
        }
        if(motor2Speed==1) {
            motor2Speed=0;
        }
        if(motor3Speed==1) {
            motor3Speed=0;
        }
        if(motor4Speed==1) {
            motor4Speed=0;
        }
        */

        if(motor1Speed==0) {
            motor1Speed=1;
        }
        if(motor2Speed==0) {
            motor2Speed=1;
        }
        if(motor3Speed==0) {
            motor3Speed=1;
        }
        if(motor4Speed==0) {
            motor4Speed=1;
        }
        // set velocity;
        this->motor1->SetVelocity(motor1Speed*3.3);
        //ThisThread::sleep_for(5);
        this->motor2->SetVelocity(motor2Speed*3.3);
        // ThisThread::sleep_for(20);
        this->motor3->SetVelocity(-1*motor3Speed*3.3);
       // ThisThread::sleep_for(20);
        this->motor4->SetVelocity(motor4Speed*3.3);
        //ThisThread::sleep_for(100);
        ThisThread::sleep_for(5);
}



void quad_omni::setAccelaration(int _acc)
{
    acc = _acc;
}

void quad_omni::setDecelaration(int _dec)
{
    dec = _dec;
}
//set the viriable input of the robot movement
void quad_omni::setVelocityX(int _vel_x)
{
    vel_x = _vel_x;
}

void quad_omni::setVelocityY(int _vel_y)
{
    vel_y = _vel_y;
}

void quad_omni::setMovementOption(int _option)
{
    option = _option;
}
    void quad_omni::setMotor1Speed(int _motor1Speed)
    {
    motor1Speed = _motor1Speed;
    }
      void quad_omni::setMotor2Speed(int _motor2Speed)
    {
    motor2Speed = _motor2Speed;
    }
    void quad_omni::setMotor3Speed(int _motor3Speed)
    {
    motor3Speed = _motor3Speed;
    }    void quad_omni::setMotor4Speed(int _motor4Speed)
    {
    motor4Speed = _motor4Speed;
    }
  void quad_omni::setRadiusW(float _radius_w){
      radius_w = _radius_w;
  };
  void quad_omni::setRadiusR(float _radius_r){
      radius_r = _radius_r;
  };
    void quad_omni::setTheta(float _theta){
      theta = _theta;
  };
    void quad_omni::setVelocityA(float _vel_a){
      vel_a = _vel_a;
  };



 int quad_omni::getMotor1Speed(){
     return motor1Speed;
 }
  int quad_omni::getMotor2Speed(){
     return motor2Speed;
 }
  int quad_omni::getMotor3Speed(){
     return motor3Speed;
 }
  int quad_omni::getMotor4Speed(){
     return motor4Speed;
 }