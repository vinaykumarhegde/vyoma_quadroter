#include "type.h"
#include "pwm.h"

#define p1 4.705e-007  
#define p2 -0.05067  
#define p3 1324

#define BACK_LEFT 2
#define FRONT_LEFT 4
#define BACK_RIGHT 5
#define FRONT_RIGHT 6

#define BACK_LEFT_MOTOR 0
#define FRONT_LEFT_MOTOR 1
#define BACK_RIGHT_MOTOR 2
#define FRONT_RIGHT_MOTOR 3

#define MIN_THRUST 550
#define MAX_THRUST 850
#define WEIGHT 2200

int pwm_val_common,pwm_val[4];
double thrust_val_common,thrust_val[4];

void stabilize(U32,double,double,double,double);
