#include "mbed.h"
#include "actiondrv.h"

static char msgsetmode[] = {0x2B,0x60,0x60,0x03,0x00,0x00,0x00,0x00};
static char msgsetacc[] = {0x23,0x83,0x60,0x00,0x80,0x96,0x98,0x00};
static char msgsetdec[] = {0x23,0x84,0x60,0x00,0x80,0x96,0x98,0x00};
static char msgsetvel[] = {0x23,0xff,0x60,0x00,0x00,0x00,0x00,0x00};
//static char heartbeat[] = {0x2B,0x17,0x10,0x00,0x64,0x00,0x00,0x00};
static char msgenable[] = {0x01,0x00};
static char enablemotor[] = {0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00};

static char setOp[] = {0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00};
static char setPreOp[] = {0x00,0x00,0x80,0x00,0x01,0x00,0x00,0x00};

DigitalOut led1(LED1);
CAN can1(PB_8, PB_9, 500000);

int rpm_to_pulse_per_s = 500*4/60;

actionDrv::actionDrv(int _id)
{
    id = _id;
       
}

void actionDrv::send(char* msg)
{
    //if(can1.write(CANMessage(0x600+ id, msg)))
    //{
    // led1 = !led1;
    //  ThisThread::sleep_for(50);    
    //}   
    
    while(!can1.write(CANMessage(0x600+ id, msg))) { ThisThread::sleep_for(50);}
}


void actionDrv::Enable(){
    //msgenable[1] = ((id >>24)& 0xFF);
    //can1.write(CANMessage(0x000, msgenable));
    // ThisThread::sleep_for(200);
    //send(enablemotor);
    
    msgenable[1] = ((id >>24)& 0xFF);
    while(!can1.write(CANMessage(0x000, msgenable)))
    ;
     ThisThread::sleep_for(300);
    send(enablemotor);
    }
void actionDrv::SetOperationalMode(){
    send(msgsetmode);
    }
void actionDrv::Configvelocity(int acc, int dec){
    
    msgsetacc[4] = ((acc ) & 0xFF);
    msgsetacc[5] = ((acc >> 8) & 0xFF);
    msgsetacc[6] = ((acc >> 16) & 0xFF);
    msgsetacc[7] = ((acc >> 24) & 0xFF);
    msgsetdec[4] = ((dec ) & 0xFF);
    msgsetdec[5] = ((dec >> 8) & 0xFF);
    msgsetdec[6] = ((dec >> 16) & 0xFF);
    msgsetdec[7] = ((dec >> 24)  & 0xFF);
    
    send(msgsetacc);
    send(msgsetdec);
    }
void actionDrv::SetVelocity(int vel){
    //vel = vel*500*4/60; // rpm to pulse per s
    vel = vel*rpm_to_pulse_per_s;
    msgsetvel[4] = ((vel ) &0xFF);
    msgsetvel[5] = ((vel >> 8) &0xFF);
    msgsetvel[6] = ((vel >> 16) &0xFF);
    msgsetvel[7] = ((vel >> 24) & 0xFF);
    send(msgsetvel);
    }
    
void actionDrv::send_mod(char* msg)
{
    //if(can1.write(CANMessage(0x600+ id, msg)))
    //{
    // led1 = !led1;    
    //}   
    
    while(!can1.write(CANMessage(0x600+ id, msg)));
}
 
void actionDrv::SetVelocity_mod(int vel){
    //vel = vel*500*4/60; // rpm to pulse per s
    vel = vel*rpm_to_pulse_per_s;
    msgsetvel[4] = ((vel ) &0xFF);
    msgsetvel[5] = ((vel >> 8) &0xFF);
    msgsetvel[6] = ((vel >> 16) &0xFF);
    msgsetvel[7] = ((vel >> 24) & 0xFF);
    send_mod(msgsetvel);
    }
    

    
void actionDrv::stop(){
    SetVelocity(0);
    
    }


