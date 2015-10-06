#ifndef __estimate_h
#define __estimate_h
#include "type.h"
#include <cross_studio_io.h>

typedef struct vector
{
 S16 x;
 S16 y;
 S16 z;
}vector; 

typedef struct angles
{
 double roll;
 double pitch;
}angles;

typedef struct angular_rate
{
 double roll_rate;
 double pitch_rate;
}angular_rate;

typedef struct pilot_input
{
 U8 throttle;
 U8 roll;
 U8 pitch;
 U8 channel3;
 U8 channel4;
}pilot_input;

double roll_x1,roll_x2,roll_y1;
double pitch_x1,pitch_x2,pitch_y1;
int flag_freefall;

void filter(angles *, angular_rate *, angles *,double);
void init_filter(angles *, angular_rate *, angles *);

#endif