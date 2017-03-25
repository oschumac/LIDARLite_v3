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



#include <ros/ros.h>

#define DEBUG 1

#include "include/LIDARLite_v3/LIDARLite_v3.h"


typedef struct LIDARdata
{
	//ros::Publisher pub;
	///geometry_msgs::AccelStamped data;
	uint16_t value;
	int32_t I2C_Handle_LIDAR;
} LIDARdata;


void ISRgetLIDAR(int gpio, int level, uint32_t tick, void* data)
{
	LIDARdata* LIDAR1 = (LIDARdata*) data;

	//ROSIMU->data.header.stamp = ros::Time::now();
	std::cout << readLIDAR(LIDAR1->I2C_Handle_LIDAR) << std::endl;

	//ROSIMU->pub.publish(ROSIMU->data);
}


int32_t main(int argc, char *argv[])
{
	
	//Setup ROS
	ros::init(argc, argv, "LIDARLite_v3");
	ros::NodeHandle node;
	//ros::Publisher IMU_pub = node.advertise<geometry_msgs::AccelStamped>("LIDAR", 1000);
	//ros::Subscriber sub = node.subscribe("LIDAR_command", 1000, IMU_commandCallback);
	//ros::Rate loop_rate(1000);

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
	
	//Start LIDAR
	LIDAR1.I2C_Handle_LIDAR = initLIDAR();
	if (LIDAR1.I2C_Handle_LIDAR < 0)
	{
		#ifdef DEBUG
		std::cout << "Failed to init LIDAR" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
	
	void* data = (void *)(&LIDAR1);
	bool isValid = ISRLIDAR(LIDAR1.I2C_Handle_LIDAR, ISRgetLIDAR,data);
	if (isValid == false)
	{
		#ifdef DEBUG
		std::cout << "Failed to start ISR" << std::endl;
		#endif
		return EXIT_FAILURE;
	}


	
	triggerOneShotLIDAR(LIDAR1.I2C_Handle_LIDAR);
	while (ros::ok())
	{
		ros::spinOnce();
		gpioDelay(1000000);
		triggerLIDAR(LIDAR1.I2C_Handle_LIDAR);
		std::cout << "Trigger" << std::endl;
	}

	
   	gpioTerminate();


	return EXIT_SUCCESS;

}



