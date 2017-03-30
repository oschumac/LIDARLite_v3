#include "include/LIDARLite_v3/DRV8825.h"



bool initDRV8825(DRV8825pin *data,uint32_t _DIR,uint32_t STEP,uint32_t M0,uint32_t M1,uint32_t M2)
{
	int32_t isValid = gpioSetMode(_DIR,PI_OUTPUT);
	if (isValid < 0)
	{
		#ifdef DRV8825DEBUG
		printf("Failed to set pin DRV8825\n");
		#endif
		return false;
	}
	
	gpioSetMode(STEP,PI_OUTPUT);
	gpioSetMode(M0,PI_OUTPUT);
	gpioSetMode(M1,PI_OUTPUT);
	gpioSetMode(M2,PI_OUTPUT);

	
	data->_DIR = _DIR;
	data->STEP = STEP;
	data->M0 = M0;
	data->M1 = M1;
	data->M2 = M2;

	bool isCorrect = resetDRV8825(data);
	if (isCorrect == false)
	{
		#ifdef DRV8825DEBUG
		printf("Failed to set reset DRV8825\n");
		#endif
		return false;
	}
	

	return true;
}



bool resetDRV8825(DRV8825pin *data)
{
	int32_t isValid = gpioWrite(data->M0, 0);
	if (isValid < 0)
	{
		return false;
	}

	gpioWrite(data->M1, 0);
	gpioWrite(data->M2, 0);
	gpioWrite(data->STEP, 0);
	gpioWrite(data->_DIR, 0);	
	data->StepMode = 1;
	data->count = 0;
	data->forward = true;
	
	return true;	
}

void setStepMode(DRV8825pin *data,uint8_t StepMode)
{
	switch (StepMode)
	{
		case 2:
			gpioWrite(data->M0, 1);
			gpioWrite(data->M1, 0);
			gpioWrite(data->M2, 0);
			data->StepMode = 2;
			break;
		case 4:	
			gpioWrite(data->M0, 0);
			gpioWrite(data->M1, 1);
			gpioWrite(data->M2, 0);
			data->StepMode = 4;
			break; 
		default:
			gpioWrite(data->M0, 0);
			gpioWrite(data->M1, 0);
			gpioWrite(data->M2, 0);
			data->StepMode = 1;
			break; 
	}
}


void stepDRV8825(DRV8825pin *data)
{
	gpioWrite(data->STEP, 1);
	gpioDelay(STEPUS);
	gpioWrite(data->STEP, 0);
	gpioDelay(STEPUS);
	
	if (data->forward)
	{
		data->count = (data->count + 1) % (REVSTEPS*(data->StepMode));
	}
	else
	{
		if (data->count == 0)
		{
			data->count = (REVSTEPS*(data->StepMode));
		}
		else
		{
			--data->count;
		}
	}
}

void dirDRV8825(DRV8825pin *data, bool direction)
{
	if (direction == DRV8825_FORWARD)
	{
		gpioWrite(data->_DIR, 0);
		data->forward = true;
		return;
	}
	else
	{
		data->forward = false;
		gpioWrite(data->_DIR, 1);
	}
}


