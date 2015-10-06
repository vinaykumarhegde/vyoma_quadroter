#include "hardware.h"
#include "lcd.h"
#include "lpc_2148.h"
#include "type.h"
#include "utils.h"
#include "pwm.h"
#include "imu.h"
#include "motor.h"
#include "filter.h"
#include "miscellaneous.h"
#include "lpc_uart.h"
#include "uart.h"
#include "pid.h"
#include "math.h"
#include "uart_isr.h"
#include "lpc_vic.h"
#include <cross_studio_io.h>

#define UART0_RX_INT_MODE

void pilot_init();

extern pilot_input pilot;

void main()
{ 
 //Variables declaration Starts
 int i=0,ch;            
 double time;
 angles unfiltered,filtered;
 angular_rate gyro;
 
 //Variable declaration ends 

 tn_arm_disable_interrupts();
 HardwareInit();  //Intializes PLL,RTC,Ports,PWM,I2C0,UART0
 itgConfig();     //Configures ITG Gyroscope 
 adxl345Config(); //Configures ADXL Accelerometer
 pilot_init();
 tn_arm_enable_interrupts();
/* calc_angles(&unfiltered);      //calculates initial angles 
 do{  
   }while(itgRead(&gyro)==-1);  //and its rates for initializing filter
 init_filter(&unfiltered,&gyro,&filtered);  //Intializes complementary filter Algorithm
 */
 //init_pid(&pid_roll,1);   //Intializes roll angle PID loop
 //init_pid(&pid_pitch,2);  //Intializes pitch angle PID loop

 filtered.roll=0;
 filtered.pitch=0;

while(1)
 {
  uart0Putch('a');
  /*ch = uart0Getch(&ch);
  debug_printf("%c\n",ch);*/
 }

 /*Run_without_feedback(1);
 delay(200000);
 Run_without_feedback(2);
 delay(200000);
 Run_without_feedback(3);
 delay(200000);
 Run_without_feedback(4);*/

 debug_printf("Start");
 start_time();            //Starts Real Time Clock of LPC2148
 for(int j=1;j<=3;j++)
 {
  Run_without_feedback(j); 
  for(i=0;i<500;i++)
  {
   calc_angles(&unfiltered);      //Gets angles from accelerometer

  do{  
    }while(itgRead(&gyro)==-1);  //Gets angular rates from Gyroscope

  get_time(&time);     //Get time from RTC
  filtered.roll = (0.9969)*(filtered.roll) +0.9969* integrate_gyro(time,gyro.roll_rate) + (3.055e-3)*unfiltered.roll;
  filtered.pitch = (0.9969)*(filtered.pitch) +0.9969* integrate_gyro(time,gyro.pitch_rate) + (3.055e-3)*unfiltered.pitch; 
  //filter(&unfiltered,&gyro,&filtered,time);  //Complementary filter gives correct angles 
 
  //stabilize_with_feedback(pilot.throttle,0,0,filtered.roll,0);  //with feedback
  SendSensorData(gyro,unfiltered,filtered);     
  delay(1000);
  }
 }
  Run_without_feedback(0);
  debug_printf("End\n");
}

void pilot_init()
{
  pilot.throttle = 0;
  pilot.roll = 0;
  pilot.pitch = 0;
}

