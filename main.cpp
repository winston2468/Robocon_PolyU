#include "./USBHostXpad/USBHostXpad.h"
#include "mbed.h"
#include "quad_omni/quad_omni.h"
#include "./INA3221/INA3221.h"
#include "./DT35/DT35.h"
#include "BufferedSerial.h"
#define RECEIVING 1;
#define TRY1 2;
#define TRY2 4;
#define TRY3 6;
#define TRY4 8;
#define TRY5 10;
#define BACKING1 3;
#define BACKING2 5;
#define BACKING3 7;
#define BACKING4 9;
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
Thread DS4BT_thread;
uint8_t DS4BT_packet[15] = {0};

 BufferedSerial pc(USBTX, USBRX);
  BufferedSerial device(PA_9, PA_10);
CAN* can1 = new CAN(PB_5, PB_6, 500000);

Thread DS4_thread;
Thread quad_omni_thread;
PwmOut servo_1(PA_5);
 int servo_curr_pw = 1500;
 int servo_max = 1500;
 int servo_min = 1220;
 int servo_backward_speed = 5;
 int servo_forward_speed = 3;
DigitalOut relay_1(PB_9,0);
volatile bool triangle, circle, cross, square;
volatile bool DPAD_NW, DPAD_W, DPAD_SW, DPAD_S, DPAD_SE, DPAD_E, DPAD_NE, DPAD_N;
volatile bool options, share, r2, l2, r1, l1;
volatile bool touchpad, touchpad_finger0, touchpad_finger1, PS;
volatile bool r3, l3;
volatile int lstick_x, lstick_y, rstick_x, rstick_y;
volatile int l2_trig, r2_trig;
volatile int buttons_l;
bool ready = 0;
 float PI = 3.14159265358979323846;
 float theator = -PI/2;
 int autoMode = 0;
 int auto_stage = 0;
 int fence_x = 4340;
//volatile int fence_y = 7000;
 int pillar_x = 2750;
 int pillar_y1 = 5630;
 int pillar_y2 = 4210;
 int key_point_x = 5000;
 int key_point_y = 6200;
 int try_spot_center_x = 5690;
 int try_spot1_center_y = 7000;
 int try_spot2_center_y = 5630;
 int try_spot3_center_y = 4210;
 int try_spot4_center_y = 2800;
 int try_spot5_center_y = 1310;
 int pass_point_center_x = 840;
 int pass_point1_center_y = 9900;
 int pass_point2_center_y = 9900;
 int pass_point3_center_y = 9690;
 int pass_point4_center_y = 9450;
 int pass_point5_center_y = 9210;
 int distance1 = 0; //y
 int distance2 = 0; //x_top
 int distance3 = 0; //x_bottom
 int changing_range_y1 = 5430; // acceptable changing range of motor movement in mm/ms (y pillar 1)
 int changing_range_y2 = 4010; // acceptable changing range of motor movement in mm/ms (y pillar 2)
 int changing_range_x1 = 2550; // acceptable changing range of motor movement in mm/ms (x pillar)
 int changing_range_x2 = 4100; // acceptable changing range of motor movement in mm/ms (x fence)
 int automote_scale = 1000; //scale up the motors' speed

quad_omni *quad_omni_class = new quad_omni(1, 2, 3, 4, can1);
DT35 *DT35_class = new DT35(PB_4, PA_8, (0x80));        //VS:0x82; SCL:0x86; SDA:0x84; GND:0x80

void setAutoMode(){
    if(autoMode == 1){
        autoMode = 0;
        quad_omni_class->setTheta(theator+PI);
    }
    if(autoMode == 0){
        autoMode = 1;
        quad_omni_class->setTheta(theator+3*PI/2);
    }
}

void servo_auto(){
    for(servo_curr_pw; servo_curr_pw > servo_min; servo_curr_pw -= servo_forward_speed){
        //pc.printf("%d\n\rservo\r\n",servo_curr_pw);
        servo_1.pulsewidth_us(servo_curr_pw); 
    }
    for(servo_curr_pw; servo_curr_pw < servo_max; servo_curr_pw += servo_backward_speed){
        //pc.printf("%d\n\rservo\r\n",servo_curr_pw);
        servo_1.pulsewidth_us(servo_curr_pw); 
    }
}

void parseDS4(int buttons, int buttons2, int stick_lx, int stick_ly,
              int stick_rx, int stick_ry, int trigger_l, int trigger_r) {

    triangle = buttons & (1 << 7);
    circle = buttons & (1 << 6);
    cross = buttons & (1 << 5);
    square = buttons & (1 << 4);

    buttons_l = buttons & 0x0f;
    DPAD_NW = buttons_l == 0x07;
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
    if (!(stick_lx > 118 && stick_lx < 136)) {
        lstick_x = stick_lx - 127;
    } 
    else {
        lstick_x = 0;
    }
    if (!(stick_ly > 118 && stick_ly < 136)) {
        lstick_y = -1*(stick_ly - 127);
    } 
    else {
        lstick_y = 0;
    }
    if (!(stick_rx > 118 && stick_rx < 136)) {
        rstick_x = stick_rx - 127;
    } 
    else {
        rstick_x = 0;
    }
    if (!(stick_ry > 118 && stick_ry < 136)) {
        rstick_y =  -1*(stick_ry - 127);
    } 
    else {
        rstick_y = 0;
    }


    l2_trig = trigger_l;
    r2_trig = trigger_r;
    relay_1 = circle;
    if (square) {
        if((auto_stage % 2) == 0){
            auto_stage++;
            setAutoMode();
        }
    }
    if(options){
        if((auto_stage % 2) == 1){
            auto_stage++;
            setAutoMode();
        }
    }
    if(share){
        autoMode = 0;
    }
    servo_curr_pw = constrain(cross * servo_backward_speed - triangle * servo_forward_speed + servo_curr_pw, servo_min, servo_max);
    //pc.printf("%d\n\rservo\r\n",servo_curr_pw);
    servo_1.pulsewidth_us(servo_curr_pw);  

}

void Parse_DS4_BT() {
/*
    if (DS4BT_packet[6]& (1 << 0)) {
    pc.printf("Triangle\r\n");
  }
  if (DS4BT_packet[6]& (1 << 1)) {
    pc.printf("Circle\r\n");
  }
  if (DS4BT_packet[6]& (1 << 2)) {
    pc.printf("Cross\r\n");
  }
  if (DS4BT_packet[6]& (1 << 3)) {
    pc.printf("Square\r\n");
  }
  if (DS4BT_packet[6]& (1 << 4)) {
    pc.printf("Up\r\n");
  }
  if (DS4BT_packet[6]& (1 << 5)) {
    pc.printf("Right\r\n");
  }
  if (DS4BT_packet[6]& (1 << 6)) {
    pc.printf("Down\r\n");
  }
  if (DS4BT_packet[6]& (1 << 7)) {
    pc.printf("Left\r\n");
  }
  if (DS4BT_packet[7]& (1 << 0)) {
    pc.printf("L1\r\n");
  }
  if (DS4BT_packet[7]& (1 << 1)) {
    pc.printf("L3\r\n");
  }
  if (DS4BT_packet[7]& (1 << 2)) {
    pc.printf("R1\r\n");
  }
  if (DS4BT_packet[7]& (1 << 3)) {
    pc.printf("R3\r\n");
  }
  if (DS4BT_packet[7]& (1 << 4)) {
    pc.printf("Share\r\n");
  }
  if (DS4BT_packet[7]& (1 << 5)) {
    pc.printf("Options\r\n");
  }
  if (DS4BT_packet[7]& (1 << 6)) {
    pc.printf("Touchpad\r\n");
  }
  if (DS4BT_packet[7]& (1 << 7)) {
    pc.printf("PS\r\n");
  }



    pc.printf("L2 %d\r\n", DS4BT_packet[4]);

    pc.printf("R2 %d\r\n", DS4BT_packet[5]);
  
  pc.printf("lstick_x %d\r\n", DS4BT_packet[0]);
  pc.printf("lstick_y %d\r\n", DS4BT_packet[1]);
  pc.printf("rstick_x %d\r\n", DS4BT_packet[2]);
  pc.printf("rstick_y %d\r\n", DS4BT_packet[3]);

if (DS4BT_packet[8]& (1 << 0)) {
    pc.printf("Touch 0\r\n");
  }
  if (DS4BT_packet[8]& (1 << 1)) {
    pc.printf("Touch 1\r\n");
  }
  pc.printf("Angle Pitch %d\r\n", DS4BT_packet[9]);
  pc.printf("Angle Roll %d\r\n", DS4BT_packet[10]);
  pc.printf("Touch 0X %d\r\n", DS4BT_packet[11]);
  pc.printf("Touch 0Y %d\r\n", DS4BT_packet[12]);
  pc.printf("Touch 1X %d\r\n", DS4BT_packet[13]);
  pc.printf("Touch 1Y %d\r\n", DS4BT_packet[14]);
  

  pc.printf("--------------------------------------------\r\n");
*/
  triangle = DS4BT_packet[6]& (1 << 0);
  circle =DS4BT_packet[6]& (1 << 1);
  cross = DS4BT_packet[6]& (1 << 2);
  square = DS4BT_packet[6]& (1 << 3);

  DPAD_NW = DS4BT_packet[6]& (1 << 5) &&DS4BT_packet[6]& (1 << 4);
  DPAD_W = DS4BT_packet[6]& (1 << 5);
  DPAD_SW = (DS4BT_packet[6]& (1 << 5) && DS4BT_packet[6]& (1 << 6));
  DPAD_S = DS4BT_packet[6]& (1 << 6);
  DPAD_SE = (DS4BT_packet[6]& (1 << 6) && DS4BT_packet[6]& (1 << 7));
  DPAD_E = DS4BT_packet[6]& (1 << 7);
  DPAD_NE = (DS4BT_packet[6]& (1 << 7) && DS4BT_packet[6]& (1 << 4));
  DPAD_N = DS4BT_packet[6]& (1 << 4);
  r3 = DS4BT_packet[7]& (1 << 3);
  l3 = DS4BT_packet[7]& (1 << 1);
  options = DS4BT_packet[7]& (1 << 5);
  share = DS4BT_packet[7]& (1 << 4);
  touchpad = DS4BT_packet[7]& (1 << 6);
  PS = DS4BT_packet[7]& (1 << 7);
  r1 = DS4BT_packet[7]& (1 << 2);
  l1 = DS4BT_packet[7]& (1 << 0);


  //Deadzone
  if (!(DS4BT_packet[0] > 118 && DS4BT_packet[0] < 136)) {
    lstick_x = DS4BT_packet[0] - 127;
  } else {
    lstick_x = 0;
  }
  if (!(DS4BT_packet[1] > 118 && DS4BT_packet[1] < 136)) {
    lstick_y = -1*(DS4BT_packet[1] - 127);
  } else {
    lstick_y = 0;
  }
  if (!(DS4BT_packet[2] > 118 && DS4BT_packet[2] < 136)) {
    rstick_x = DS4BT_packet[2] - 127;
  } else {
    rstick_x = 0;
  }
  if (!(DS4BT_packet[3] > 118 && DS4BT_packet[3] < 136)) {
    rstick_y =  -1*(DS4BT_packet[3] - 127);
  } else {
    rstick_y = 0;
  }
    
  l2_trig = DS4BT_packet[4];
  r2_trig = DS4BT_packet[5];
    l2_trig >=0 ? l2=l2_trig  : l2=0;
    r2_trig >=0 ? r2=r2_trig  : r2=0;
    relay_1 = circle;
    if (square) {
        if((auto_stage % 2) == 0){
            auto_stage++;
            setAutoMode();
        }
    }
    if(options){
        if((auto_stage % 2) == 1){
            auto_stage++;
            setAutoMode();
        }
    }
    if(share){
        autoMode = 0;
    }
    servo_curr_pw = constrain(cross * servo_backward_speed - triangle * servo_forward_speed + servo_curr_pw, servo_min, servo_max);
    //pc.printf("%d\n\rservo\r\n",servo_curr_pw);
    servo_1.pulsewidth_us(servo_curr_pw);  

}

void DS4BT_task() {

  while (1) {
    if (!device.readable())
      ;
    if (device.getc() == 'D') {
      while (!device.readable())
        ;
      if (device.getc() == 'S') {

        while (!device.readable())
          ;
        if (device.getc() == '4') {
                ready=1;
          for (int i = 0; i < 15; i++) {
            while (!device.readable());
              DS4BT_packet[i] = device.getc();
            
          }




        }
      }
    }
     if(ready){
 Parse_DS4_BT();
 }
   // ThisThread::sleep_for(15);
 }

 
}
/* 
// functions:if button pressed is true -> print

void showbuttons() {
    if (triangle) {
        pc.printf("triangle\r\n");
    }
    if (circle) {
        pc.printf("circle\r\n");
    }
    if (cross) {
        pc.printf("cross\r\n");
    }
    if (square) {
        pc.printf("square\r\n");
    }
    if (DPAD_NW) {
        pc.printf("DPAD_NW\r\n");
    }
    if (DPAD_W) {
        pc.printf("DPAD_W\r\n");
    }
    if (DPAD_SW) {
        pc.printf("DPAD_SW\r\n");
    }
    if (DPAD_S) {
        pc.printf("DPAD_S\r\n");
    }
    if (DPAD_SE) {
        pc.printf("DPAD_SE\r\n");
    }
    if (DPAD_E) {
        pc.printf("DPAD_E\r\n");
    }
    if (DPAD_NE) {
        pc.printf("DPAD_NE\r\n");
    }
    if (DPAD_N) {
        pc.printf("DPAD_N\r\n");
    }
    if (r3) {
        pc.printf("r3\r\n");
    }
    if (l3) {
        pc.printf("l3\r\n");
    }
    if (options) {
        pc.printf("options\r\n");
    }
    if (share) {
        pc.printf("share\r\n");
    }
    if (l1) {
        pc.printf("l1\r\n");
    }
    if (r1) {
        pc.printf("r1\r\n");
    }

    if (r2) {
        pc.printf("r2 %d\r\n", r2_trig);
    }
    if (l2) {
        pc.printf("l2 %d\r\n", l2_trig);
    }
    pc.printf("lstick_x %d\r\n", lstick_x);
    pc.printf("lstick_y %d\r\n", lstick_y);
    pc.printf("rstick_x %d\r\n", rstick_x);
    pc.printf("rstick_y %d\r\n", rstick_y);
    pc.printf("--------------------------------------------\r\n");
}
*/



// attached function, USBHostXpad onUpdate
void onXpadEvent(int buttons, int buttons2, int stick_lx, int stick_ly,
                 int stick_rx, int stick_ry, int trigger_l, int trigger_r) {
    /* 
    pc.printf("DS4: %02x %02x %-5d %-5d %-5d %-5d %02x %02x\r\n", buttons,
    buttons2, stick_lx, stick_ly, stick_rx, stick_ry, trigger_l, trigger_r);
    */
    parseDS4(buttons, buttons2, stick_lx, stick_ly, stick_rx, stick_ry, trigger_l,
             trigger_r);
}

void xpad_task() {
    USBHostXpad xpad;
    xpad.attachEvent(&onXpadEvent);
    while (1) {
        while (!xpad.connect()) {
            // This sleep_for can be removed
            parseDS4(0,0,127,127,127,127,0,0);
            ThisThread::sleep_for(1000);
        }
        while (xpad.connected()) {
        }
    }
}

void quad_omni_task() {
    quad_omni_class->motorInitialization();
    //pc.printf("%d\r\n", autoMode);
    while (1) {
        if (l1) {
            quad_omni_class->setMovementOption(1);
        } 
        else if (r1) {
            quad_omni_class->setMovementOption(2);
        } 
        else if (l2) {
            quad_omni_class->setMovementOption(3);
        } 
        else if (r2) {
            quad_omni_class->setMovementOption(4);
        } 
        else {
            quad_omni_class->setMovementOption(0);
        }
        if(distance1 >= pillar_y1){
            if((distance1 - changing_range_y1) >= DT35_class->getBusVoltage(1, 1)){
                distance1 = (DT35_class->getBusVoltage(1, 1) + pillar_y1);
            }
            else{
                distance1 = DT35_class->getBusVoltage(1, 1);
            }
        }
        else if(distance1 >= pillar_y2){
            if((distance1 - changing_range_y2) >= DT35_class->getBusVoltage(1, 1)){
                distance1 = (DT35_class->getBusVoltage(1, 1) + pillar_y2);
            }
            else{
                distance1 = DT35_class->getBusVoltage(1, 1);
            }
        }
        else{
            distance1 = (DT35_class->getBusVoltage(1, 1));
        }
        if(distance2 >= fence_x){
            if((distance2 - changing_range_x2) >= DT35_class->getBusVoltage(1, 2)){
                distance2 = (DT35_class->getBusVoltage(1, 2) + fence_x);
            }
            else{
                distance2 = DT35_class->getBusVoltage(1, 2);
            }
        }
        else if(distance2 >= pillar_x){
            if((distance2 - changing_range_x1) >= DT35_class->getBusVoltage(1, 2)){
                distance2 = (DT35_class->getBusVoltage(1, 2) + pillar_x);
            }
            else{
                distance2 = DT35_class->getBusVoltage(1, 2);
            }
        }
        else{
            distance2 = (DT35_class->getBusVoltage(1, 2));
        }
        if(distance3 >= fence_x){
            if((distance3 - changing_range_x2) >= DT35_class->getBusVoltage(1, 3)){
                distance3 = (DT35_class->getBusVoltage(1, 3) + fence_x);
            }
            else{
                distance3 = DT35_class->getBusVoltage(1, 3);
            }
        }
        else if(distance3 >= pillar_x){
            if((distance3 - changing_range_x1) >= DT35_class->getBusVoltage(1, 3)){
                distance3 = (DT35_class->getBusVoltage(1, 3) + pillar_x);
            }
            else{
                distance3 = DT35_class->getBusVoltage(1, 3);
            }
        }
        else{
            distance3 = (DT35_class->getBusVoltage(1, 3));
        }
        /* 
        pc.printf("auto:%d   ", auto_stage);
        pc.printf("mode:%d   ", autoMode);
        pc.printf("CH1:%dV   ", distance1);
        pc.printf("CH2:%dV   ", distance2);
        pc.printf("CH3:%dV   \r\n", distance3);
        */
        if(autoMode==0){
            // show what buttons are pressed every 0.5s
            //showbuttons();
            // This sleep_for can be removed
            //ThisThread::sleep_for(100);
            
            quad_omni_class->setVelocityX(lstick_x * 7000);
            quad_omni_class->setVelocityY(lstick_y * 7000);
                        
            if(rstick_x != 0 || rstick_y != 0)
            {
                quad_omni_class->setVelocityX(rstick_x * 2000);
                quad_omni_class->setVelocityY(rstick_y * 2000);
            }

            if (DPAD_N) {
                quad_omni_class->setVelocityY(300000);
            }
            else if (DPAD_S) {
                quad_omni_class->setVelocityY(-300000);
            }
            else if (DPAD_E) {
                quad_omni_class->setVelocityX(300000);
            }
            else if (DPAD_W) {
                quad_omni_class->setVelocityX(-300000);
            }
        }
        else if(autoMode==1){
            /*          
            if(distance3 > distance2){
                quad_omni_class->setMovementOption(6);
            }
            else if(distance3 < distance2){
                quad_omni_class->setMovementOption(5);
            }
            else {
                quad_omni_class->setMovementOption(0);
            }
            */

            if(auto_stage == 1){
                if(distance1 < pass_point1_center_y){
                    quad_omni_class->setVelocityX(0 * automote_scale);
                    quad_omni_class->setVelocityY(128 * automote_scale);
                }
                else if(distance1 >= pass_point1_center_y){
                    if(distance2 < pass_point_center_x){
                        quad_omni_class->setVelocityX(-128 * automote_scale);
                        quad_omni_class->setVelocityY(0 * automote_scale);
                    }    
                }
                if((distance2 >= pass_point_center_x)&&(distance1 >= pass_point1_center_y)){
                    quad_omni_class->setVelocityX(0);
                    quad_omni_class->setVelocityY(0);
                    autoMode = 0;
                    quad_omni_class->setTheta(theator+PI);
                }
            }
            else if(auto_stage == 2){
                if(distance1 > key_point_y){
                    quad_omni_class->setVelocityX(0 * automote_scale);
                    quad_omni_class->setVelocityY(-128 * automote_scale);
                }
                else if(distance1 <= key_point_y){
                    if(distance2 < key_point_x){
                        quad_omni_class->setVelocityX(-128 * automote_scale);
                        quad_omni_class->setVelocityY(0 * automote_scale);
                    }    
                }
                if((distance2 >= key_point_x)&&(distance1 <= key_point_y)){
                    if(distance1 < try_spot1_center_y){
                        quad_omni_class->setVelocityX(0 * automote_scale);
                        quad_omni_class->setVelocityY(128 * automote_scale);
                    }
                    else if(distance1 >= try_spot1_center_y){
                        if(distance2 < try_spot_center_x){
                            quad_omni_class->setVelocityX(-128 * automote_scale);
                            quad_omni_class->setVelocityY(0 * automote_scale);
                        }    
                    }
                    if((distance2 >= try_spot_center_x)&&(distance1 >= try_spot1_center_y)){
                        quad_omni_class->setVelocityX(0);
                        quad_omni_class->setVelocityY(0);
                        servo_auto();
                        autoMode = 0;
                        quad_omni_class->setTheta(theator+PI);
                    }
                }
            }
            else if(auto_stage == 3){
                if(distance2 > key_point_x){
                    quad_omni_class->setVelocityX(128 * automote_scale);
                    quad_omni_class->setVelocityY(0 * automote_scale);
                }
                else if(distance2 <= key_point_x){
                    if(distance1 > key_point_y){
                        quad_omni_class->setVelocityX(0 * automote_scale);
                        quad_omni_class->setVelocityY(-128 * automote_scale);
                    }    
                }
                if((distance2 <= key_point_x)&&(distance1 <= key_point_y)){
                    if(distance2 > pass_point_center_x){
                        quad_omni_class->setVelocityX(128 * automote_scale);
                        quad_omni_class->setVelocityY(0 * automote_scale);
                    }
                    else if(distance2 <= pass_point_center_x){
                        if(distance1 < pass_point2_center_y){
                            quad_omni_class->setVelocityX(0 * automote_scale);
                            quad_omni_class->setVelocityY(128 * automote_scale);
                        }    
                    }
                    if((distance2 <= pass_point_center_x)&&(distance1 >= pass_point2_center_y)){
                        quad_omni_class->setVelocityX(0);
                        quad_omni_class->setVelocityY(0);
                        autoMode = 0;
                        quad_omni_class->setTheta(theator+PI);
                    }
                }
            }
            else if(auto_stage == 4){
                if(distance1 > key_point_y){
                    quad_omni_class->setVelocityX(0 * automote_scale);
                    quad_omni_class->setVelocityY(-128 * automote_scale);
                }
                else if(distance1 <= key_point_y){
                    if(distance2 < key_point_x){
                        quad_omni_class->setVelocityX(-128 * automote_scale);
                        quad_omni_class->setVelocityY(0 * automote_scale);
                    }    
                }
                if((distance2 >= key_point_x)&&(distance1 <= key_point_y)){
                    if(distance1 > try_spot2_center_y){
                        quad_omni_class->setVelocityX(0 * automote_scale);
                        quad_omni_class->setVelocityY(-128 * automote_scale);
                    }
                    else if(distance1 <= try_spot2_center_y){
                        if(distance2 < try_spot_center_x){
                            quad_omni_class->setVelocityX(-128 * automote_scale);
                            quad_omni_class->setVelocityY(0 * automote_scale);
                        }    
                    }
                    if((distance2 >= try_spot_center_x)&&(distance1 <= try_spot2_center_y)){
                        quad_omni_class->setVelocityX(0);
                        quad_omni_class->setVelocityY(0);
                        servo_auto();
                        autoMode = 0;
                        quad_omni_class->setTheta(theator+PI);
                    }
                }
            }
            else if(auto_stage == 5){
                if(distance2 > key_point_x){
                    quad_omni_class->setVelocityX(128 * automote_scale);
                    quad_omni_class->setVelocityY(0 * automote_scale);
                }
                else if(distance2 <= key_point_x){
                    if(distance1 < key_point_y){
                        quad_omni_class->setVelocityX(0 * automote_scale);
                        quad_omni_class->setVelocityY(128 * automote_scale);
                    }    
                }
                if((distance2 <= key_point_x)&&(distance1 >= key_point_y)){
                    if(distance2 > pass_point_center_x){
                        quad_omni_class->setVelocityX(128 * automote_scale);
                        quad_omni_class->setVelocityY(0 * automote_scale);
                    }
                    else if(distance2 <= pass_point_center_x){
                        if(distance1 < pass_point3_center_y){
                            quad_omni_class->setVelocityX(0 * automote_scale);
                            quad_omni_class->setVelocityY(128 * automote_scale);
                        }    
                    }
                    if((distance2 <= pass_point_center_x)&&(distance1 >= pass_point3_center_y)){
                        quad_omni_class->setVelocityX(0);
                        quad_omni_class->setVelocityY(0);
                        autoMode = 0;
                        quad_omni_class->setTheta(theator+PI);
                    }
                }
            }
            else if(auto_stage == 6){
                if(distance1 > key_point_y){
                    quad_omni_class->setVelocityX(0 * automote_scale);
                    quad_omni_class->setVelocityY(-128 * automote_scale);
                }
                else if(distance1 <= key_point_y){
                    if(distance2 < key_point_x){
                        quad_omni_class->setVelocityX(-128 * automote_scale);
                        quad_omni_class->setVelocityY(0 * automote_scale);
                    }    
                }
                if((distance2 >= key_point_x)&&(distance1 <= key_point_y)){
                    if(distance1 > try_spot3_center_y){
                        quad_omni_class->setVelocityX(0 * automote_scale);
                        quad_omni_class->setVelocityY(-128 * automote_scale);
                    }
                    else if(distance1 <= try_spot3_center_y){
                        if(distance2 < try_spot_center_x){
                            quad_omni_class->setVelocityX(-128 * automote_scale);
                            quad_omni_class->setVelocityY(0 * automote_scale);
                        }    
                    }
                    if((distance2 >= try_spot_center_x)&&(distance1 <= try_spot3_center_y)){
                        quad_omni_class->setVelocityX(0);
                        quad_omni_class->setVelocityY(0);
                        servo_auto();
                        autoMode = 0;
                        quad_omni_class->setTheta(theator+PI);
                    }
                }
            }
            else if(auto_stage == 7){
                if(distance2 > key_point_x){
                    quad_omni_class->setVelocityX(128 * automote_scale);
                    quad_omni_class->setVelocityY(0 * automote_scale);
                }
                else if(distance2 <= key_point_x){
                    if(distance1 < key_point_y){
                        quad_omni_class->setVelocityX(0 * automote_scale);
                        quad_omni_class->setVelocityY(128 * automote_scale);
                    }    
                }
                if((distance2 <= key_point_x)&&(distance1 >= key_point_y)){
                    if(distance2 > pass_point_center_x){
                        quad_omni_class->setVelocityX(128 * automote_scale);
                        quad_omni_class->setVelocityY(0 * automote_scale);
                    }
                    else if(distance2 <= pass_point_center_x){
                        if(distance1 < pass_point4_center_y){
                            quad_omni_class->setVelocityX(0 * automote_scale);
                            quad_omni_class->setVelocityY(128 * automote_scale);
                        }    
                    }
                    if((distance2 <= pass_point_center_x)&&(distance1 >= pass_point4_center_y)){
                        quad_omni_class->setVelocityX(0);
                        quad_omni_class->setVelocityY(0);
                        autoMode = 0;
                        quad_omni_class->setTheta(theator+PI);
                    }
                }
            }
            else if(auto_stage == 8){
                if(distance1 > key_point_y){
                    quad_omni_class->setVelocityX(0 * automote_scale);
                    quad_omni_class->setVelocityY(-128 * automote_scale);
                }
                else if(distance1 <= key_point_y){
                    if(distance2 < key_point_x){
                        quad_omni_class->setVelocityX(-128 * automote_scale);
                        quad_omni_class->setVelocityY(0 * automote_scale);
                    }    
                }
                if((distance2 >= key_point_x)&&(distance1 <= key_point_y)){
                    if(distance1 > try_spot4_center_y){
                        quad_omni_class->setVelocityX(0 * automote_scale);
                        quad_omni_class->setVelocityY(-128 * automote_scale);
                    }
                    else if(distance1 <= try_spot4_center_y){
                        if(distance2 < try_spot_center_x){
                            quad_omni_class->setVelocityX(-128 * automote_scale);
                            quad_omni_class->setVelocityY(0 * automote_scale);
                        }    
                    }
                    if((distance2 >= try_spot_center_x)&&(distance1 <= try_spot4_center_y)){
                        quad_omni_class->setVelocityX(0);
                        quad_omni_class->setVelocityY(0);
                        servo_auto();
                        autoMode = 0;
                        quad_omni_class->setTheta(theator+PI);
                    }
                }
            }
            else if(auto_stage == 9){
                if(distance2 > key_point_x){
                    quad_omni_class->setVelocityX(128 * automote_scale);
                    quad_omni_class->setVelocityY(0 * automote_scale);
                }
                else if(distance2 <= key_point_x){
                    if(distance1 < key_point_y){
                        quad_omni_class->setVelocityX(0 * automote_scale);
                        quad_omni_class->setVelocityY(128 * automote_scale);
                    }    
                }
                if((distance2 <= key_point_x)&&(distance1 >= key_point_y)){
                    if(distance2 > pass_point_center_x){
                        quad_omni_class->setVelocityX(128 * automote_scale);
                        quad_omni_class->setVelocityY(0 * automote_scale);
                    }
                    else if(distance2 <= pass_point_center_x){
                        if(distance1 < pass_point5_center_y){
                            quad_omni_class->setVelocityX(0 * automote_scale);
                            quad_omni_class->setVelocityY(128 * automote_scale);
                        }    
                    }
                    if((distance2 <= pass_point_center_x)&&(distance1 >= pass_point5_center_y)){
                        quad_omni_class->setVelocityX(0);
                        quad_omni_class->setVelocityY(0);
                        autoMode = 0;
                        quad_omni_class->setTheta(theator+PI);
                    }
                }
            }
            else if(auto_stage == 10){
               if(distance1 > key_point_y){
                    quad_omni_class->setVelocityX(0 * automote_scale);
                    quad_omni_class->setVelocityY(-128 * automote_scale);
                }
                else if(distance1 <= key_point_y){
                    if(distance2 < key_point_x){
                        quad_omni_class->setVelocityX(-128 * automote_scale);
                        quad_omni_class->setVelocityY(0 * automote_scale);
                    }    
                }
                if((distance2 >= key_point_x)&&(distance1 <= key_point_y)){
                    if(distance1 > try_spot5_center_y){
                        quad_omni_class->setVelocityX(0 * automote_scale);
                        quad_omni_class->setVelocityY(-128 * automote_scale);
                    }
                    else if(distance1 <= try_spot5_center_y){
                        if(distance2 < try_spot_center_x){
                            quad_omni_class->setVelocityX(-128 * automote_scale);
                            quad_omni_class->setVelocityY(0 * automote_scale);
                        }    
                    }
                    if((distance2 >= try_spot_center_x)&&(distance1 <= try_spot5_center_y)){
                        quad_omni_class->setVelocityX(0);
                        quad_omni_class->setVelocityY(0);
                        servo_auto();
                        autoMode = 0;
                        quad_omni_class->setTheta(theator+PI);
                    }
                }
            }
            //ThisThread::sleep_for(100);
        }
        //pc.printf("%d %d %d %d \r\n",quad_omni_class->getMotor1Speed(),quad_omni_class->getMotor2Speed(),quad_omni_class->getMotor3Speed(),quad_omni_class->getMotor4Speed() );
        quad_omni_class->motorUpdate();
        //pc.printf("--------------------------------------------\r\n");
        ThisThread::sleep_for(20);
    }
}

void DT35_initialazation(){
    //setup
    DT35_class->DT35_initialization(1, 3);
    //printf("INA3221:   FID:%d   UID:%d    Mode:%d\r\n",DT35_class->getManufacturerID(1),DT35_class->getDieID(1),DT35_class->getConfiguration(1));
    //printf("INA3221:   FID:%d   UID:%d    Mode:%d\r\n",DT35_class->getManufacturerID(2),DT35_class->getDieID(2),DT35_class->getConfiguration(2));
    //printf("INA3221:   FID:%d   UID:%d    Mode:%d\r\n",DT35_class->getManufacturerID(3),DT35_class->getDieID(3),DT35_class->getConfiguration(3));
}

int main() {
   device.baud(115200);
pc.baud(115200);

   
    pc.printf("--------------------------------------------\r\n");
    DT35_initialazation();
    quad_omni_thread.start(callback(quad_omni_task));
    //DS4_thread.start(callback(xpad_task));
    servo_1.period_us (2500);
    servo_1.pulsewidth_us(500);
     DS4BT_thread.start(callback(DS4BT_task));
    while (1) {
    }
    return 0;
}
