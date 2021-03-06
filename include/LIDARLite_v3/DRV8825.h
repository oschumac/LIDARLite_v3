#include <pigpio.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>
#include <math.h> 
#include <stdint.h>
#include <stdbool.h>

//Debug print out
#define DRV8825DEBUG 1
#define STEPUS 50
#define DRV8825_FORWARD 0
#define DRV8825_BACKWARD 1
#define REVSTEPS 200

typedef struct DRV8825pin
{
	uint32_t _DIR;
	uint32_t STEP;
	uint32_t M0;
	uint32_t M1;
	uint32_t M2;
	uint8_t StepMode;
	uint32_t count;
	bool forward;
} DRV8825pin;



bool initDRV8825(DRV8825pin *data,  uint32_t _DIR,uint32_t STEP,uint32_t M0,uint32_t M1,uint32_t M2);

bool resetDRV8825(DRV8825pin *data);

void setStepMode(DRV8825pin *data,uint8_t StepMode);

void stepDRV8825(DRV8825pin *data);

void dirDRV8825(DRV8825pin *data, bool direction);
