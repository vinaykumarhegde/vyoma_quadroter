#include "miscellaneous.h"
#include "filter.h"
#include "imu.h"
#include "math.h"
#include "lpc_uart.h"
#include "uart.h"
#include "motor.h"

void disp(U16 value)
{
 int i;
 char temp;
 char array[5]={48,48,48,48,0};
 for(i=4; i!=0; i--)
 {
  temp = value%10;
  array[i-1]+=temp;
  value/=10;
 }
 lcd_putstring(0,array);
}

void calc_angles(angles *tilt)
{
 S16 temp[3];
 vector accel;
 if(adxl345Read(temp)!=-1)
    { 
     accel.x=temp[0];
     accel.y=temp[1];
     accel.z=temp[2];
     uart0Putch(accel.x);
     uart0Putch(accel.y);
     uart0Putch(accel.z);
    // debug_printf("\n\nX = %d\nY = %d\nZ = %d\n",accel.x,accel.y,accel.z);

     tilt->roll=atan2(accel.x,accel.z)*180/PI;   //Roll angle is around Y - axis
     tilt->pitch=atan2(accel.y,accel.z)*180/PI;  //Pitch angle is around X - axis
     if(0)
     {
      debug_printf("x:%d\t",accel.x); 
      debug_printf("y:%d\t",accel.y); 
      debug_printf("z:%d\t",accel.z); 
      debug_printf("Unfiltered Roll: %f\t",tilt->roll); 
      debug_printf("Unfiltered Pitch: %f\n",tilt->pitch); 
     }  
     if(accel.z<40)
     flag_freefall=1;
     else
     flag_freefall=0;
    }
}  


void uart_display(S16 *a)
{
 U16 temp;
 char c;
 temp=(0x8000&a[0])>>15;
 if(temp==1)
 {
  a[0]=(~a[0])+1;
  uart0Putch('-');
 }
 c=((a[0]/10000)%10)+48;
 uart0Putch(c);
 c=((a[0]/1000)%10)+48; 
 uart0Putch(c);
 c=((a[0]/100)%10)+48;
 uart0Putch(c);
 c=((a[0]/10)%10)+48;
 uart0Putch(c);
 c=(a[0]%10)+48;
 uart0Putch(c);
 uart0Putch('\n');
 uart0Putch('\r');
}

double integrate_gyro(double time,double sensor_output)
{
  int i=0;
  static double a[N]={0};
  double angle=0,sum1=0,sum2=0;
  
 
  
  for(i=0;i<N;i++)
  a[i]=a[i+1];
  a[N-1]=sensor_output;
  for(i=1;i<N;i+=2)
   sum1+=a[i];
  for(i=2;i<N-1;i+=2)
   sum2+=a[i];

  angle=(time/3)*((a[0]+a[N-1])+4*sum1+2*sum2);

  return(angle);
}

void GetUserData(pilot_input *pilot)
{
  int i=0;
  U8 ch;
  for(i=0;i<5;i++)
  {
    ch = uart0Getch(&ch);
    if(ch>=0x40 && ch<0x50) pilot->throttle=ch-0x40;
    else if(ch>=0x50 && ch<0x60) pilot->roll=ch-0x50;
    else if(ch>=0x60 && ch<0x70) pilot->pitch=ch-0x60;
    else if(ch>=0x70 && ch<0x80) pilot->channel3=ch-0x70;
    else if(ch>=0x80 && ch<0x90) pilot->channel4=ch-0x80;
  }
  if(1)
  {
   debug_printf("Pilot_Throttle:%d\n",pilot->throttle);
   //debug_printf("Pilot_Roll:%d\t",pilot.roll);
   //debug_printf("Pilot_Pitch:%d\n",pilot.pitch);
  }

}

void SendSensorData(angular_rate gyro,angles unfiltered,angles filtered)
{ 
 conv_to_char(unfiltered.roll);
 //debug_printf("%f\n",unfiltered.roll);
 conv_to_char(unfiltered.pitch);
 //debug_printf("%f\n",unfiltered.pitch);
 conv_to_char(gyro.roll_rate);
 //debug_printf("%f\n",gyro.roll_rate);
 conv_to_char(gyro.pitch_rate);
 //debug_printf("%f\n",gyro.pitch_rate);
 conv_to_char(filtered.roll);
 //debug_printf("%f\n",filtered.roll);
 conv_to_char(filtered.pitch);
 //debug_printf("%f\n",filtered.pitch);
}

void conv_to_char(double number)
{
 int temp1,temp2;
 if(number<0)
  temp2=(int)((number-ceil(number))*100);
 else
  temp2=(int)((number-floor(number))*100);

  temp1=(int)(number);
 //debug_printf("%d.%d\t",temp1,temp2);
 uart0Putch(temp1);
 uart0Putch(temp2);
}

void start_time()
{
 CCR |=(1<<1);//CTCRST=1
 CCR &=~(1<<1);//CTCRST=0
 SEC=0;
}

void get_time(double *time)
{
 *time=SEC+(CTC/32768.0);
 //debug_printf("time=%f\t",*time);
 CCR |=(1<<1);//CTCRST=1
 CCR &=~(1<<1);//CTCRST=0
 SEC=0;
}

void Run_without_feedback(U8 pilot_throttle)
{
 pwm_val_common=63500+(3888*pilot_throttle);
 thrust_val_common=p1*((double)pwm_val_common*(double)pwm_val_common) + p2*(double)pwm_val_common + p3;
 for(int i=0;i<4;i++)
 {
  pwm_val[i]=pwm_val_common;
  thrust_val[i]=thrust_val_common;
  if(0)
  debug_printf("pwm[%d]= %d\t",i,pwm_val[i]);
 }
 if(0)
 debug_printf("\n");
 load_pwm(SINGLE,BACK_LEFT,960000,pwm_val[BACK_LEFT_MOTOR],0);//correct PIN31 PWM 2
 load_pwm(SINGLE,FRONT_LEFT,960000,pwm_val[FRONT_LEFT_MOTOR],0);//correct PIN33 PWM 4
 load_pwm(SINGLE,BACK_RIGHT,960000,pwm_val[BACK_RIGHT_MOTOR],0);//correct PIN1 PWM 5   
 load_pwm(SINGLE,FRONT_RIGHT,960000,pwm_val[FRONT_RIGHT_MOTOR],0);//correct PIN34 PWM 6
}