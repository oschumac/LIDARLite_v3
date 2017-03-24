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
#define LIDARDEBUG 1


#define LIDARI2CADDR 0x62


#define LIDARSLEEPUS 20000

//ISR
#define LIDARISRPIN 4
#define LIDARISRTYPE FALLING_EDGE 
#define LIDARISRTIMEOUT 0

//Registers
#define ACQ_COMMAND 0x00
#define SIG_COUNT_VAL 0x02
#define ACQ_CONFIG_REG 0x04
#define THRESHOLD_BYPASS 0x00
#define MEASURE_DELAY 0x45
#define OUTER_LOOP_COUNT 0x11
#define LIDAR_VALUE 0x8f



int32_t initLIDAR();
bool resetLIDAR(int32_t I2C_Handle);
bool configureLIDAR(int32_t I2C_Handle);
bool ISRLIDAR(int32_t I2C_Handle, gpioISRFuncEx_t f,void* data);
void triggerOneShotLIDAR(int32_t I2C_Handle);
void triggerLIDAR(int32_t I2C_Handle);
uint16_t readLIDAR(int32_t I2C_Handle);



