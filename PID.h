#include "adc.h"
#include "hardware.h"
#include "lcd.h"
#include "lpc_2148.h"
#include "type.h"
#include "utils.h"
#include "pwm.h"

#define kp_roll 1
#define ki_roll 0
#define kd_roll 0

#define kp_pitch 1
#define ki_pitch 0
#define kd_pitch 0

struct parameters
{
 double kp;
 double ki;
 double kd;
 double sum_error;
 double prev_error;
 double max_sum_error;
};

struct parameters pid_roll,pid_pitch;

void init_pid(struct parameters *ptr,U8 type);
double pid(struct parameters *ptr,double ref,double y);
