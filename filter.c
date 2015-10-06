#include "type.h"
#include "math.h"
#include "filter.h"

#define K 0.007 // K - bandwidth of filter (1/sec). Need to tune this to match sensor performance

// inputs:
// gyro - gyro output (deg/sec)
// unfiltered - accelerometer input to filter (deg)

// outputs:
// filtered - complementary filter output (and output of second integrator), (deg)
//            This also needs to be saved each iteration.

// initialization (when on ground after initial power-up)
void init_filter(angles *unfiltered, angular_rate *gyro, angles *filtered)
{ 
 filtered->roll=unfiltered->roll;
 filtered->pitch=unfiltered->pitch;
 roll_y1=-gyro->roll_rate;
 pitch_y1=-gyro->pitch_rate;   //account for gyro bias
}


// while flying - do this every iteration through the control loop
void filter(angles *unfiltered, angular_rate *gyro, angles *filtered,double time)
{
// if quad is in free-fall, or if attitude is too large, accel can't be used to determine angle
//	so just integrate the gyro in this case. Actual numbers are just guesses at this point
 if ((flag_freefall==1)|| filtered->roll > 30 || filtered->roll < -30)
	unfiltered->roll = filtered->roll; 
 if ((flag_freefall==1)|| filtered->pitch > 30 || filtered->pitch < -30)
	unfiltered->pitch = filtered->pitch;

 //  here's the filter for roll
 roll_x1 = (double)((unfiltered->roll - filtered->roll)*K*K);
 roll_y1 = time*(roll_x1) + roll_y1;
 roll_x2 = roll_y1 + (double)((unfiltered->roll - filtered->roll)*2*K) + gyro->roll_rate;
 filtered->roll = time*(roll_x2) + filtered->roll;

 //  here's the filter for pitch
 pitch_x1 = (double)((unfiltered->pitch - filtered->pitch)*K*K);
 pitch_y1 = time*(pitch_x1) + pitch_y1;
 pitch_x2 = pitch_y1 + (double)((unfiltered->pitch - filtered->pitch)*2*K) + gyro->pitch_rate;
 filtered->pitch = time*(pitch_x2) + filtered->pitch;

 if(1)
 {
  debug_printf("Filtered_roll=%f\t",filtered->roll);
  debug_printf("Filtered_pitch=%f\n",filtered->pitch);
 }
} 


















/*
vector RGyro,curr_Est,pre_Est;    //RGyro==temp position acc to gyro, Estimates are the best fit values
angles curr_angles,pre_angles;       //Angles which represents estimated position

int signof(double num)      //to find sign for RGyro.z
{
 if(num<0)                 
 return -1;
 else
 return 1;
}

angles estimate(vector Acc,angles Gyro,U32 curr)  //passed values are readings from aceel,and gyro
{
 int sign;
 double temp,R;    
 if(curr==0)          //for first time beleive aceelerometer and use the same
 curr_Est=Acc;
 else                     //from next onwards, calculate best estimate using previous ones and gyro
 {  
  sign=signof(pre_Est.z);
  curr_angles.XZ=pre_angles.XZ+(Gyro.XZ*T);       //calculating present angles using previuos and gyro rate
  curr_angles.YZ=pre_angles.YZ+(Gyro.YZ*T);
  curr_angles.XY=pre_angles.XY+(Gyro.XY*T);
  RGyro.x=sin(curr_angles.XZ)/(sqrt(1+pow(cos(curr_angles.XZ),2)*pow(tan(curr_angles.YZ),2))); //estimating position acc.to gyro
  RGyro.y=sin(curr_angles.YZ)/(sqrt(1+pow(cos(curr_angles.YZ),2)*pow(tan(curr_angles.XZ),2)));
  temp=1-pow(RGyro.x,2)-pow(RGyro.y,2);
  RGyro.z=sqrt(temp);
  curr_Est.x=(Acc.x+(RGyro.x*WGyro))/(1+WGyro);      //using both aceel and gyro's position, giving certain weights and 
  curr_Est.y=(Acc.y+(RGyro.y*WGyro))/(1+WGyro);     //estimate the best position
  curr_Est.z=(Acc.z+(RGyro.z*WGyro))/(1+WGyro);
  R=sqrt(pow(curr_Est.x,2)+pow(curr_Est.y,2)+pow(curr_Est.z,2));
  curr_Est.x/=R;                 //Normalizing
  curr_Est.y/=R;
  curr_Est.z/=R;
 }
 curr_angles.XZ=atan2(curr_Est.x,curr_Est.z);   //calculating angles from estimate of position
 curr_angles.YZ=atan2(curr_Est.y,curr_Est.z);
 curr_angles.XY=atan2(curr_Est.x,curr_Est.y);
 pre_angles=curr_angles;        //pevious angles == current angles for next calculations
 pre_Est=curr_Est;              //previous estimate == current estimate for next calculations
 return curr_angles;
}

*/