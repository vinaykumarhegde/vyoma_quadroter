#include "adc.h"
#include "hardware.h"
#include "lcd.h"
#include "lpc_2148.h"
#include "type.h"
#include "utils.h"
#include "pwm.h"
#include "PID.h"

void init_pid(struct parameters *ptr,U8 type)       //Intialize the parameters at start
{
 switch(type)
 {
  case 1:
  ptr->kp=kp_roll;
  ptr->ki=ki_roll;
  ptr->kd=kd_roll;
  ptr->sum_error=0;
  ptr->prev_error=0;
  ptr->max_sum_error=100;
  break;
  case 2:
  ptr->kp=kp_pitch;
  ptr->ki=ki_pitch;
  ptr->kd=kd_pitch;
  ptr->sum_error=0;
  ptr->prev_error=0;
  ptr->max_sum_error=100;
  break;
 }
}

double pid(struct parameters *ptr,double ref,double current)   //pid algo here
{
 double error,u;
 error=ref-current;             //error value calculated
 ptr->sum_error+=error;    //sumerror updated 
 
 //below is the pid expression
 u=((ptr->kp)*error)+((ptr->ki)*(ptr->sum_error))+((ptr->kd)*(error-(ptr->prev_error))); 
 ptr->prev_error=error; //update previous error
 return u;   //return control input;
}