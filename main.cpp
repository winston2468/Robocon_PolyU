
#include "USBHostXpad.h"
#include "mbed.h"
#include "quad_omni/quad_omni.h"
CAN* can1 = new CAN(PB_5, PB_6, 500000);
Serial pc(USBTX, USBRX);
Thread DS4_thread;
Thread quad_omni_thread;
volatile int triangle, circle, cross, square;
volatile int DPAD_NW, DPAD_W, DPAD_SW, DPAD_S, DPAD_SE, DPAD_E, DPAD_NE, DPAD_N;
volatile int r3, l3, options, share, r2, l2, r1, l1;
volatile int lstick_x, lstick_y, rstick_x, rstick_y;
volatile int l2_trig, r2_trig;
volatile int buttons_l;

void parseDS4(int buttons, int buttons2, int stick_lx, int stick_ly,
              int stick_rx, int stick_ry, int trigger_l, int trigger_r) {

  triangle = buttons & (1 << 7);
  circle = buttons & (1 << 6);
  cross = buttons & (1 << 5);
  square = buttons & (1 << 4);
  /* 

  buttons_l = buttons & 0x0f;
  DPAD_NW = buttons_l == 0x07;
  DPAD_W = buttons_l == 0x06;
  DPAD_SW = buttons_l == 0x05;
  DPAD_S = buttons_l == 0x04;
  DPAD_SE = buttons_l == 0x03;
  DPAD_E = buttons_l == 0x02;
  DPAD_NE = buttons_l == 0x01;
  DPAD_N = buttons_l == 0x00;

*/
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
  } else {
    lstick_x = 0;
  }
  if (!(stick_ly > 118 && stick_ly < 136)) {
    lstick_y = -1*(stick_ly - 127);
  } else {
    lstick_y = 0;
  }
  if (!(stick_rx > 118 && stick_rx < 136)) {
    rstick_x = stick_rx - 127;
  } else {
    rstick_x = 0;
  }
  if (!(stick_ry > 118 && stick_ry < 136)) {
    rstick_y =  -1*(stick_ry - 127);
  } else {
    rstick_y = 0;
  }

  l2_trig = trigger_l;
  r2_trig = trigger_r;
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
  // pc.printf("DS4: %02x %02x %-5d %-5d %-5d %-5d %02x %02x\r\n", buttons,
  // buttons2, stick_lx, stick_ly, stick_rx, stick_ry, trigger_l, trigger_r);
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

      // show what buttons are pressed every 0.5s
      //showbuttons();
      // This sleep_for can be removed
      //ThisThread::sleep_for(100);
    }
  }
}

void quad_omni_task() {
  quad_omni *quad_omni_class = new quad_omni(1, 2, 3, 4, can1);
  quad_omni_class->motorInitialization();
  while (1) {
    
    quad_omni_class->setVelocityX(lstick_x * 4500);
    quad_omni_class->setVelocityY(lstick_y * 4500);
    if (DPAD_N) {
        quad_omni_class->setVelocityY(300000);
    }
    else if (DPAD_S) {
        quad_omni_class->setVelocityY(-300000);
    }
    if (DPAD_E) {
        quad_omni_class->setVelocityX(300000);
    }
    else if (DPAD_W) {
        quad_omni_class->setVelocityX(-300000);
    }
    if (l1|| rstick_x <0) {
      quad_omni_class->setMovementOption(1);

    } else if (r1|| rstick_x >0) {
      quad_omni_class->setMovementOption(2);

    } else {
      quad_omni_class->setMovementOption(0);
    }
    pc.printf("%d %d %d %d \r\n",quad_omni_class->getMotor1Speed(),quad_omni_class->getMotor2Speed(),quad_omni_class->getMotor3Speed(),quad_omni_class->getMotor4Speed() );
    quad_omni_class->motorUpdate();
    
  }
}

int main() {
    
  quad_omni_thread.start(callback(quad_omni_task));
  // motorInitialization(); //Must be on first line in function due to some
  // wried timing problems with the motor controller
  pc.baud(115200);
  pc.printf("--------------------------------------------\r\n");

  DS4_thread.start(callback(xpad_task));

  while (1) {
  }
}
