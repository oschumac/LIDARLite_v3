#include <pigpio.h>
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <cstdint>
#include <cinttypes>
#include <unistd.h>
#include <math.h> 
#include <fstream>
#include <vector>

#include <ros/ros.h>

#define DEBUG 1

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

#include "include/LIDARLite_v3/LIDARLite_v3.h"
#include "include/LIDARLite_v3/DRV8825.h"

typedef struct LIDARdata
{
	ros::Publisher pub;
	sensor_msgs::LaserScan LIDARdata;
	uint16_t value;
	int32_t I2C_Handle_LIDAR;
} LIDARdata;


/*
void ISRgetLIDAR(int gpio, int level, uint32_t tick, void* data)
{
	LIDARdata* LIDAR1 = (LIDARdata*) data;

	//ROSIMU->data.header.stamp = ros::Time::now();
	std::cout << "LIDARISR: " << readLIDAR(LIDAR1->I2C_Handle_LIDAR) << std::endl;

	//ROSIMU->pub.publish(ROSIMU->data);
}
*/



int32_t main(int argc, char *argv[])
{
	
	//Setup ROS
	ros::init(argc, argv, "LIDARLite_v3");
	ros::NodeHandle node;
	ros::Publisher LIDAR_pub = node.advertise<sensor_msgs::LaserScan>("scan", 500);
	//ros::Subscriber sub = node.subscribe("LIDAR_command", 1000, IMU_commandCallback);
	ros::Rate loop_rate(500);


	#ifdef DEBUG
	std::cout << "Starting" << std::endl;
	#endif
	
	//Start pigpio
	if (gpioInitialise() < 0)
	{
		#ifdef DEBUG
		std::cout << "Failed to start" << std::endl;
		#endif
		return EXIT_FAILURE;
	}


	LIDARdata LIDAR1;
	LIDAR1.pub = LIDAR_pub;
	LIDAR1.LIDARdata.header.frame_id = "laser";
	LIDAR1.LIDARdata.range_min = LIDAR_RANGE_MIN;
	LIDAR1.LIDARdata.range_max = LIDAR_RANGE_MAX;	
	LIDAR1.LIDARdata.angle_increment = (1.8/4.0)*(M_PI/180);
	LIDAR1.LIDARdata.angle_min = - M_PI;
	
	
	//Start LIDAR
	LIDAR1.I2C_Handle_LIDAR = initLIDAR();
	if (LIDAR1.I2C_Handle_LIDAR < 0)
	{
		#ifdef DEBUG
		std::cout << "Failed to init LIDAR" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
	
	
	//LIDAR ISR
	/*
	void* data = (void *)(&LIDAR1);
	bool isValid = ISRLIDAR(LIDAR1.I2C_Handle_LIDAR, ISRgetLIDAR,data);
	if (isValid == false)
	{
		#ifdef DEBUG
		std::cout << "Failed to start ISR" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
	*/
	

	//Stepper motor
	DRV8825pin NEMA17;
	bool isValid = initDRV8825(&NEMA17, 27, 22,10,9,11);
	if (isValid == false)
	{
		#ifdef DEBUG
		std::cout << "Failed to init DRV8825" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
	setStepMode(&NEMA17,4);
	dirDRV8825(&NEMA17, DRV8825_FORWARD);

	float range = 0.0;
	double dt = 0.0;
	ros::Time startScan,startTrigger,endTrigger;
	while (ros::ok())
	{
		startTrigger = ros::Time::now();
		if (NEMA17.count == 0)
		{
			startScan = startTrigger;
		}

		if ((NEMA17.count % 100) == 0)
		{
			triggerOneShotECLIDAR(LIDAR1.I2C_Handle_LIDAR);
		}
		else
		{
			triggerOneShotLIDAR(LIDAR1.I2C_Handle_LIDAR);
		}
		pollLIDAR(LIDAR1.I2C_Handle_LIDAR);
		
		endTrigger = ros::Time::now();
		 
		range = readLIDAR(LIDAR1.I2C_Handle_LIDAR);
		dt = ((endTrigger - startTrigger).toSec());
		if ((dt > 1.0) || (NEMA17.count == 799)) 
		{
			if (NEMA17.count == 799)
			{
				LIDAR1.LIDARdata.ranges.push_back(range);
			}
			
			if (LIDAR1.LIDARdata.ranges.size() > 0)
			{
				LIDAR1.LIDARdata.header.stamp = startScan;
				LIDAR1.LIDARdata.angle_max = LIDAR1.LIDARdata.angle_min + ((((float)NEMA17.count) + 1.0)*((2.0*M_PI)/(800.0))) ;
				LIDAR1.LIDARdata.scan_time = (float)((endTrigger - startScan).toSec());
				LIDAR1.LIDARdata.time_increment = (float)((LIDAR1.LIDARdata.scan_time)/(((double)NEMA17.count) + 1.0));
				LIDAR1.pub.publish(LIDAR1.LIDARdata);
			}
			
			if (NEMA17.count == 799)
			{
				LIDAR1.I2C_Handle_LIDAR = initLIDAR();
				dirDRV8825(&NEMA17, DRV8825_BACKWARD);
				gpioDelay(1000);
				while ( NEMA17.count > 0 )
				{
					stepDRV8825(&NEMA17);
					gpioDelay(1000);
				}
				dirDRV8825(&NEMA17, DRV8825_FORWARD);
				gpioDelay(1000);
			}
			
			LIDAR1.LIDARdata.ranges.clear();
			LIDAR1.LIDARdata.angle_min = (NEMA17.count % 800 )*((2.0*M_PI)/(800.0)) - M_PI;
			continue;
		}
		stepDRV8825(&NEMA17);
		LIDAR1.LIDARdata.ranges.push_back(range);
	}

	
   	gpioTerminate();


	return EXIT_SUCCESS;

}

