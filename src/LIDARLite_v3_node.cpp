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

#include <geometry_msgs/PointStamped.h>

#include "include/LIDARLite_v3/LIDARLite_v3.h"
#include "include/LIDARLite_v3/DRV8825.h"

typedef struct LIDARdata
{
	ros::Publisher pub;
	geometry_msgs::PointStamped LIDARdata;
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
	ros::Publisher LIDAR_pub = node.advertise<geometry_msgs::PointStamped>("LIDAR", 500);
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

	int count;
	while (ros::ok())
	{
		triggerOneShotECLIDAR(LIDAR1.I2C_Handle_LIDAR);
		pollLIDAR(LIDAR1.I2C_Handle_LIDAR);
		
		LIDAR1.LIDARdata.header.stamp = ros::Time::now();
		LIDAR1.LIDARdata.point.x = (double)readLIDAR(LIDAR1.I2C_Handle_LIDAR);
		LIDAR1.LIDARdata.point.y = (double)NEMA17.count;
		LIDAR1.pub.publish(LIDAR1.LIDARdata);
		
		//stepDRV8825(&NEMA17);
		
		for (count=0;count < 99;++count)
		{
			triggerOneShotLIDAR(LIDAR1.I2C_Handle_LIDAR);
			pollLIDAR(LIDAR1.I2C_Handle_LIDAR);
			
			LIDAR1.LIDARdata.header.stamp = ros::Time::now();
			LIDAR1.LIDARdata.point.x = (double)readLIDAR(LIDAR1.I2C_Handle_LIDAR);
			LIDAR1.LIDARdata.point.y = (double)NEMA17.count;
			LIDAR1.pub.publish(LIDAR1.LIDARdata);


			//stepDRV8825(&NEMA17);
			
		}
	

	}

	
   	gpioTerminate();


	return EXIT_SUCCESS;

}



