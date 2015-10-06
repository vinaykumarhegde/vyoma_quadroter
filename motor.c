#include "motor.h"
#include "pid.h"
#include "math.h"
#include "miscellaneous.h"

void stabilize_with_feedback(U32 pilot_throttle,double r_roll,double r_pitch,double p_roll,double p_pitch)
{
 
 double roll_control,pitch_control;
 double tr,tp;
 int sat_flag=0,i;
 double total_lift,lift_diff;

 roll_control=pid(&pid_roll,r_roll,p_roll);
// pitch_control=pid(&pid_pitch,r_pitch,p_pitch);
 debug_printf("u=%f\t",roll_control);
 pwm_val_common=63500+(3888*pilot_throttle);
 debug_printf("user_pwm=%d\t",pwm_val_common);
 thrust_val_common=p1*((double)pwm_val_common*(double)pwm_val_common) + p2*(double)pwm_val_common + p3; 
 debug_printf("user_thrust=%f\t",thrust_val_common);
 tr=thrust_val_common*((1/cos(roll_control*PI/180))-1);
 //tp=thrust_val_common*((1/cos(pitch_control*PI/180))-1);
 debug_printf("tr=%f\t",tr);

 if((p_roll-r_roll)>0)
 {
  if((thrust_val[BACK_RIGHT_MOTOR]+=tr)>MAX_THRUST)
   sat_flag=1;
  if((thrust_val[FRONT_RIGHT_MOTOR]+=tr)>MAX_THRUST)
   sat_flag=1;
  if(sat_flag)
  {
   thrust_val[BACK_RIGHT_MOTOR]-=tr;
   thrust_val[FRONT_RIGHT_MOTOR]-=tr;
   thrust_val[FRONT_LEFT_MOTOR]-=2*tr;
   thrust_val[BACK_LEFT_MOTOR]-=2*tr;
  }
  else
  {
   thrust_val[BACK_LEFT_MOTOR]-=tr;
   thrust_val[FRONT_LEFT_MOTOR]-=tr;
  }
  total_lift=(thrust_val[BACK_LEFT_MOTOR]+thrust_val[FRONT_LEFT_MOTOR]+thrust_val[BACK_RIGHT_MOTOR]+thrust_val[FRONT_RIGHT_MOTOR])*cos(roll_control*PI/180);
  if(total_lift<WEIGHT)
   lift_diff=(WEIGHT-total_lift)/4;
  else
   lift_diff=0;
  for(i=0;i<4;i++)
  {
   thrust_val[i]+=(lift_diff/cos(roll_control*PI/180));
   debug_printf("thrust[%d]=%f\t",i,thrust_val[i]);
  }
 }
 else
 {
  if((thrust_val[FRONT_LEFT_MOTOR]+=tr)>MAX_THRUST)
   sat_flag=1;
  if((thrust_val[BACK_LEFT_MOTOR]+=tr)>MAX_THRUST)
   sat_flag=1;
  if(sat_flag)
  {
   thrust_val[BACK_LEFT_MOTOR]-=tr;
   thrust_val[FRONT_LEFT_MOTOR]-=tr;
   thrust_val[BACK_RIGHT_MOTOR]-=2*tr;
   thrust_val[FRONT_RIGHT_MOTOR]-=2*tr;
  }
  else
  {
   thrust_val[FRONT_RIGHT_MOTOR]-=tr;
   thrust_val[BACK_RIGHT_MOTOR]-=tr;
  }
  debug_printf("lift1=%f\t",thrust_val[0]);
  debug_printf("lift2=%f\t",thrust_val[1]);
  debug_printf("lift3=%f\t",thrust_val[2]);
  debug_printf("lift4=%f\t",thrust_val[3]);
  total_lift=(thrust_val[0]+thrust_val[1]+thrust_val[2]+thrust_val[3])*cos(roll_control*PI/180);
  debug_printf("total_lift=%f\t",total_lift);
  if(total_lift<WEIGHT)
   lift_diff=(WEIGHT-total_lift)/4;
  else
   lift_diff=0;
  for(i=0;i<4;i++)
  {
   thrust_val[i]+=(lift_diff/cos(roll_control*PI/180));
   debug_printf("thrust[%d]=%f\t",i,thrust_val[i]);
  }
 }
 for(i=0;i<4;i++)
 {
  pwm_val[i]=53847+(10627*sqrt(.7523+(thrust_val[i]*0.01882)));
  debug_printf("pwm[%d]=%d\t",i,pwm_val[i]);
 }
 debug_printf("\n");
 load_pwm(SINGLE,FRONT_RIGHT,960000,pwm_val[FRONT_RIGHT_MOTOR],0);//correct PIN31
 load_pwm(SINGLE,FRONT_LEFT,960000,pwm_val[FRONT_LEFT_MOTOR],0);//correct PIN33
 load_pwm(SINGLE,BACK_RIGHT,960000,pwm_val[BACK_RIGHT_MOTOR],0);//correct PIN1
 load_pwm(SINGLE,BACK_LEFT,960000,pwm_val[BACK_LEFT_MOTOR],0);//correct PIN34
 sat_flag=0;
}