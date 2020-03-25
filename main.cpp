
#include "mbed.h"
#include "USBHostXpad.h"

#include "actiondrv.h"
#include "Timer.h"

#define PI 3.14159265359


Serial pc(USBTX, USBRX);
Thread DS4;
volatile int triangle, circle, cross, square;
volatile int DPAD_NW, DPAD_W, DPAD_SW, DPAD_S, DPAD_SE, DPAD_E, DPAD_NE, DPAD_N;
volatile int r3, l3, options, share, r2, l2, r1, l1;
volatile int lstick_x, lstick_y, rstick_x, rstick_y;
volatile int l2_trig, r2_trig;


//action1 use id 1, refer to the action 
//action2 use id 2, refer to the action 
//action3 use id 3, refer to the action 
//action4 use id 4, refer to the action 
actionDrv action1(1); 
actionDrv action2(2); 
actionDrv action3(3); 
actionDrv action4(4); 

//moters' speed(in rpm)
float motor1Speed = 1000.0;
float motor2Speed = 1000.0;
float motor3Speed = 1000.0;
float motor4Speed = 1000.0;

//viriable for the formula calaulation
float radius_w = 20.0;          // radius of the wheel
float radius_r = 100.0;         // radius of the robot from the wheel to the center
float theator = PI / 4;         // current angle of the robot
float vel_a = 0.0;              // angular velocity needed of the turning 
float counter = 0.0;            // counter for returning to the original angle
float vel_x = 0.0;              // Vx
float vel_y = 0.0;              // Vy
float angle_per_unit = 0.01;    // the moved angle for each count, now is one count in every 0.02s when turning
int option = 0;                 // 0 -- line movement; 1 -- turn left; 2 -- turn right

//acceleration and deceleration
int acc = 1000000;
int dec = 1000000;

//Ticker object refer to Ticker API
Ticker motorUpdater;

//--function--------------------------------------------------------------------
//Initialization HF motor
void motorInitialization()
{
    //refer to manual for more detail 
    action1.Enable();
    wait(0.1);
    action1.SetOperationalMode();
    wait(0.1); 
    action1.Configvelocity(acc, dec);
    wait(0.1);
    
    //set speed to 0
    action1.stop();

    action2.Enable();
    wait(0.1);
    action2.SetOperationalMode();
    wait(0.1); 
    action2.Configvelocity(acc, dec);
    wait(0.1);
    
    //set speed to 0
    action2.stop();

    action3.Enable();
    wait(0.1);
    action3.SetOperationalMode();
    wait(0.1); 
    action3.Configvelocity(acc, dec);
    wait(0.1);
    
    //set speed to 0
    action3.stop();

    action4.Enable();
    wait(0.1);
    action4.SetOperationalMode();
    wait(0.1); 
    action4.Configvelocity(acc, dec);
    wait(0.1);
    
    //set speed to 0
    action4.stop();
}

//set the viriable input of the robot movement
void setMovementInput(int _vel_x, int _vel_y, int _option)
{
    vel_x = _vel_x;
    vel_y = _vel_y;
    option = _option;
}

//function that will be run repeatedly
//i.e constanly update velocity
void motorUpdate()
{
        // line movement; if the angle does not equal to the original one, turn it back when moving
        if(option == 0)
        {
            if(counter == 0)
            {
                motor1Speed = (vel_y * cos(theator + (PI / 4)) - vel_x * sin(theator + (PI / 4))) / radius_w;
                motor2Speed = (vel_y * cos(theator + (3 * PI / 4)) - vel_x * sin(theator + (3 * PI / 4))) / radius_w;
                motor3Speed = (vel_y * cos(theator + (5 * PI / 4)) - vel_x * sin(theator + (5 * PI / 4))) / radius_w;
                motor4Speed = (vel_y * cos(theator + (7 * PI / 4)) - vel_x * sin(theator + (7 * PI / 4))) / radius_w;
            }
            else if(counter > 0)
            {
                theator += (angle_per_unit * counter);
                if(theator >=  PI)
                {
                    theator = PI - theator;
                    counter -= (2 * PI / angle_per_unit);
                    counter += 2;
                }
                motor1Speed = (vel_y * cos(theator + (PI / 4)) - vel_x * sin(theator + (PI / 4)) - vel_a * radius_r) / radius_w;
                motor2Speed = (vel_y * cos(theator + (3 * PI / 4)) - vel_x * sin(theator + (3 * PI / 4)) - vel_a * radius_r) / radius_w;
                motor3Speed = (vel_y * cos(theator + (5 * PI / 4)) - vel_x * sin(theator + (5 * PI / 4)) - vel_a * radius_r) / radius_w;
                motor4Speed = (vel_y * cos(theator + (7 * PI / 4)) - vel_x * sin(theator + (7 * PI / 4)) - vel_a * radius_r) / radius_w;
                counter--;
            }
            else if(counter < 0)
            {
                theator -= (angle_per_unit * counter * -1);
                motor1Speed = (vel_y * cos(theator + (PI / 4)) - vel_x * sin(theator + (PI / 4)) + vel_a * radius_r) / radius_w;
                motor2Speed = (vel_y * cos(theator + (3 * PI / 4)) - vel_x * sin(theator + (3 * PI / 4)) + vel_a * radius_r) / radius_w;
                motor3Speed = (vel_y * cos(theator + (5 * PI / 4)) - vel_x * sin(theator + (5 * PI / 4)) + vel_a * radius_r) / radius_w;
                motor4Speed = (vel_y * cos(theator + (7 * PI / 4)) - vel_x * sin(theator + (7 * PI / 4)) + vel_a * radius_r) / radius_w;
                counter++;
            }
        }
        // turn left
        else if(option == 1)
        {
            motor1Speed = vel_a * radius_r / radius_w;
            motor2Speed = motor1Speed;
            motor3Speed = motor1Speed;
            motor4Speed = motor1Speed;
            counter++;
        }
        // turn right
        else if(option == 2)
        {
            motor1Speed = -1 * vel_a * radius_r / radius_w;
            motor2Speed = motor1Speed;
            motor3Speed = motor1Speed;
            motor4Speed = motor1Speed;
            counter--;
        }

        // set velocity;
        action1.SetVelocity(motor1Speed);
        action2.SetVelocity(motor2Speed);
        action3.SetVelocity(motor3Speed);
        action4.SetVelocity(motor4Speed);
        wait(0.005);
}





void parseDS4(int buttons, int buttons2, int stick_lx, int stick_ly,
              int stick_rx, int stick_ry, int trigger_l, int trigger_r) {

  triangle = buttons & (1 << 7);
  circle = buttons & (1 << 6);
  cross = buttons & (1 << 5);
  square = buttons & (1 << 4);
  int buttons_l = buttons&0x0f;
  DPAD_NW= buttons_l == 0x07;
  DPAD_W = buttons_l == 0x06;
  DPAD_SW = buttons_l == 0x05;
  DPAD_S = buttons_l == 0x04;
  DPAD_SE = buttons_l == 0x03;
  DPAD_E = buttons_l == 0x02;
  DPAD_NE = buttons_l == 0x01;
  DPAD_N = buttons_l == 0x00;
  r3 = buttons2 & (1 << 7);
  l3 = buttons2 & (1 << 6);
  options = buttons2 & (1 << 5);
  share = buttons2 & (1 << 4);
  r2 = buttons2 & (1 << 3);
  l2 = buttons2 & (1 << 2);
  r1 = buttons2 & (1 << 1);
  l1 = buttons2 & (1 << 0);
  if (!(stick_lx > 123 && stick_lx < 131) ) {
    lstick_x = stick_lx - 127;
  } else {
    lstick_x = 0;
  }
  if (!(stick_ly > 123 && stick_ly < 131) ) {
    lstick_y = stick_ly - 127;
  } else {
    lstick_y = 0;
  }
  if (!(stick_rx > 123 && stick_rx < 131) ) {
    rstick_x = stick_rx - 127;
  } else {
    rstick_x = 0;
  }
  if (!(stick_ry > 123 && stick_ry < 131)) {
    rstick_y = stick_ry - 127;
  } else {
    rstick_y = 0;
  }

  l2_trig = trigger_l;
  r2_trig = trigger_r;
  
  if(l1)
  {
      option = 1;
  }
  else if(r1){
      option = 2;
  }
  else {
      setMovementInput(lstick_x, lstick_y, 0);
  }
  

}
//functions:if button pressed is true -> print
void showbuttons()
{
     
    if(triangle){pc.printf("triangle\r\n");}
    if(circle){pc.printf("circle\r\n");}
    if(cross){pc.printf("cross\r\n");}
    if(square){pc.printf("square\r\n");}
    if(DPAD_NW){ pc.printf("DPAD_NW\r\n");}
    if(DPAD_W){ pc.printf("DPAD_W\r\n");}
    if(DPAD_SW){ pc.printf("DPAD_SW\r\n");}
    if(DPAD_S){ pc.printf("DPAD_S\r\n");}
    if(DPAD_SE){ pc.printf("DPAD_SE\r\n");}
    if(DPAD_E){ pc.printf("DPAD_E\r\n");}
    if(DPAD_NE){ pc.printf("DPAD_NE\r\n");}
    if(DPAD_N){ pc.printf("DPAD_N\r\n");}
    if(r3){pc.printf("r3\r\n");}
    if(l3){pc.printf("l3\r\n");}
    if(options){pc.printf("options\r\n");}
    if(share){pc.printf("share\r\n");} 

    if(l1){pc.printf("l1\r\n");}
    if(r1){pc.printf("r1\r\n");}

    if(r2){pc.printf("r2 %d\r\n", r2_trig);}
    if(l2){pc.printf("l2 %d\r\n", l2_trig);}
    pc.printf("lstick_x %d\r\n", lstick_x);
    pc.printf("lstick_y %d\r\n", lstick_y);
    pc.printf("rstick_x %d\r\n", rstick_x);
    pc.printf("rstick_y %d\r\n", rstick_y);
  pc.printf("--------------------------------------------\r\n");
}

//attached function, USBHostXpad onUpdate
void onXpadEvent(int buttons, int buttons2, int stick_lx, int stick_ly,
                 int stick_rx, int stick_ry, int trigger_l, int trigger_r) {
 // pc.printf("DS4: %02x %02x %-5d %-5d %-5d %-5d %02x %02x\r\n", buttons, buttons2, stick_lx, stick_ly, stick_rx, stick_ry, trigger_l, trigger_r);
 parseDS4(buttons,  buttons2,  stick_lx,  stick_ly,
                  stick_rx,  stick_ry,  trigger_l,  trigger_r);
}
void xpad_task()
{
    USBHostXpad xpad;
    xpad.attachEvent(&onXpadEvent);
    while (1) {

        while (!xpad.connect()) {
            //This sleep_for can be removed
            ThisThread::sleep_for(100);
        }


        while (xpad.connected()) {

            //show what buttons are pressed every 0.5s
            showbuttons();
            //This sleep_for can be removed
            ThisThread::sleep_for(100);
        }
    }
}

int main()
{
    pc.baud(115200);
    pc.printf("--------------------------------------------\r\n");
    DS4.start(callback(xpad_task));

        //Initialization for hf motor (id: 1, id: 2, id: 3, id: 4)
    motorInitialization();
    
    //setMovementInput(1000, 1000, 0);        <-- used for set the movement viriable input; see the function above for details

    //.attach(&|<fruction>|,interval(in seconds))
    //p.s. 1:if you want to know more refer to Ticker API 
    //p.s. 2:the interval should be MORE then the function( motorUpdate()) running time for one.

    motorUpdater.attach(&motorUpdate, 0.02);


    while (1) {

    }
}
