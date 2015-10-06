#include "lcd.h"
#include "lpc_2148.h"
#include "type.h"
#include "utils.h"
#include <cross_studio_io.h>

#define PI 3.14159265
#define N 8 //no of samples taken for integration (must be even)


void disp(U16);
void calc_angles();
double integrate_gyro(double time,double sensor_output);
void GetUserData();
void SendSensorData();
void start_time();
void get_time(double *);
void Run_without_feedback(U8);
void conv_to_char(double);