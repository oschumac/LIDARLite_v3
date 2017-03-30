#include "include/LIDARLite_v3/LIDARLite_v3.h"

int32_t initLIDAR()
{
	int32_t I2C_Handle = i2cOpen(1,LIDARI2CADDR,0);
	if (I2C_Handle < 0)
	{
		#ifdef LIDARDEBUG
		printf("Failed to start I2C\n");
		#endif
		return EXIT_FAILURE;
	}

	bool isValid = resetLIDAR(I2C_Handle);
	if (isValid < 0)
	{
		#ifdef LIDARDEBUG
		printf("Failed to reset\n");
		#endif
		return EXIT_FAILURE;
	}

	isValid = configureLIDAR(I2C_Handle);
	if (isValid < 0)
	{
		#ifdef LIDARDEBUG
		printf("Failed to configure\n");
		#endif
		return EXIT_FAILURE;
	}
	return I2C_Handle;
}

bool resetLIDAR(int32_t I2C_Handle)
{

	if (I2C_Handle < 0)
	{
		return false;
	}
	
	int32_t isValid  = gpioSetISRFuncEx(LIDARISRPIN,LIDARISRTYPE,0,NULL,NULL);
	if (isValid < 0)
	{
		return false;
	}
		
	isValid = i2cWriteByteData(I2C_Handle,ACQ_COMMAND,0x00);
	if (isValid < 0)
	{
		return false;
	}
	gpioSleep(PI_TIME_RELATIVE, 0, LIDARSLEEPUS);
	
	return true;
}

bool configureLIDAR(int32_t I2C_Handle)
{
	if (I2C_Handle < 0)
	{
		return false;
	}
	
	//Balanced mode
	i2cWriteByteData(I2C_Handle,SIG_COUNT_VAL, 0b10000000);
	//Interrupt mode
	//i2cWriteByteData(I2C_Handle,ACQ_CONFIG_REG, 0b0101001);
	i2cWriteByteData(I2C_Handle,ACQ_CONFIG_REG, 0b0101000);

	
	i2cWriteByteData(I2C_Handle,THRESHOLD_BYPASS, 0b0);

	gpioSleep(PI_TIME_RELATIVE, 0, LIDARSLEEPUS);
	//500Hz
	i2cWriteByteData(I2C_Handle,MEASURE_DELAY, 0x04);

	//Trigger 1 shot
	i2cWriteByteData(I2C_Handle,OUTER_LOOP_COUNT,0x01);

	
	gpioSleep(PI_TIME_RELATIVE, 0, LIDARSLEEPUS);
	return true;
}


bool ISRLIDAR(int32_t I2C_Handle, gpioISRFuncEx_t f,void* data)
{
	if (I2C_Handle < 0)
	{
		return false;
	}
	
	if (f == NULL)
	{
		return false;
	}

	//Set ISR
	if (gpioSetISRFuncEx(LIDARISRPIN,LIDARISRTYPE,LIDARISRTIMEOUT,f,data) != SETGPIOISROK)
	{
		return false;
	}

	gpioSetPullUpDown(LIDARISRPIN, PI_PUD_DOWN);
		
	gpioSleep(PI_TIME_RELATIVE, 0, LIDARSLEEPUS);
	return true;
}

void triggerOneShotECLIDAR(int32_t I2C_Handle)
{
	//Trigger 1 shot
	//i2cWriteByteData(I2C_Handle,OUTER_LOOP_COUNT,0x01);

	//Measurement with bias correction
	i2cWriteByteData(I2C_Handle,ACQ_COMMAND,0x04);
}

void triggerOneShotLIDAR(int32_t I2C_Handle)
{
	//Trigger 1 shot
	//i2cWriteByteData(I2C_Handle,OUTER_LOOP_COUNT,0x01);

	//Measurement with bias correction
	i2cWriteByteData(I2C_Handle,ACQ_COMMAND,0x03);
}



void triggerLIDAR(int32_t I2C_Handle)
{
	//Trigger 99 shots
	i2cWriteByteData(I2C_Handle,OUTER_LOOP_COUNT,0x63);

	//Measurement without correction
	i2cWriteByteData(I2C_Handle,ACQ_COMMAND,0x03);
}

uint16_t readLIDAR(int32_t I2C_Handle)
{
	char buf[2];
	i2cReadI2CBlockData(I2C_Handle,LIDAR_VALUE,buf,2);
	return (uint16_t)(  ((uint8_t)buf[0]) << 8 | ((uint8_t)buf[1]) );	
}


void pollLIDAR(int32_t I2C_Handle)
{
	while ((( (uint8_t)i2cReadByteData(I2C_Handle,STATUS)) & 0b1) == 1)
	{
		gpioDelay(10);
	}
}


