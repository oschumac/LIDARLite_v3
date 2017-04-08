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



int32_t main(int argc, char *argv[])
{
	
	//Setup ROS
	ros::init(argc, argv, "LIDARLite_v3");
	ros::NodeHandle node;
	ros::Publisher LaserScan_pub = node.advertise<sensor_msgs::LaserScan>("scan", 500);
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

	//Start LIDAR
	int32_t I2C_Handle_LIDAR = initLIDAR();
	if (LIDAR1.I2C_Handle_LIDAR < 0)
	{
		#ifdef DEBUG
		std::cout << "Failed to init LIDAR" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
	

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
	setStepMode(&NEMA17,1);
	dirDRV8825(&NEMA17, DRV8825_FORWARD);

	sensor_msgs::LaserScan LIDARdata;
	LIDARdata.header.frame_id = "laser";
	LIDARdata.range_min = LIDAR_RANGE_MIN;
	LIDARdata.range_max = LIDAR_RANGE_MAX;
	LIDARdata.angle_increment = NEMA17.angle_increment;
	LIDARdata.angle_min = NEMA17.current_angle;

	float range = 0.0;
	ros::Time startScan,endTrigger;
	while (ros::ok())
	{
		std::cout << "c: " << NEMA17.count << " a:" << NEMA17.current_angle << std::endl;
		startTrigger = ros::Time::now();
		while (NEMA.current_angle < M_PI)
		{
			if ((NEMA17.count % 100) == 0)
			{
				triggerOneShotECLIDAR(I2C_Handle_LIDAR);
			}
			else
			{
				triggerOneShotLIDAR(I2C_Handle_LIDAR);
			}
			pollLIDAR(I2C_Handle_LIDAR);
			range = readLIDAR(I2C_Handle_LIDAR);
			std::cout << "c: " << NEMA17.count << " a:" << NEMA17.current_angle << " r:" << range << std::endl;
			stepDRV8825(&NEMA17);
			LIDARdata.ranges.push_back(range);
			std::cout << "c: " << NEMA17.count << " a:" << NEMA17.current_angle << std::endl;
		}
		endTrigger = ros::Time::now();
		LIDARdata.header.stamp = startScan;
		LIDARdata.angle_max = NEMA17.current_angle - NEMA17.angle_increment;
		LIDARdata.scan_time = (float)((endTrigger - startScan).toSec());
		LIDARdata.time_increment = (float)((LIDARdata.scan_time)/(((float)NEMA17.count) + 1.0));
		LaserScan_pub.publish(LIDARdata);
		
		
		dirDRV8825(&NEMA17, DRV8825_BACKWARD);
		gpioDelay(1000);
		NEMA17.current_angle = M_PI;
		LIDARdata.ranges.clear();
		LIDARdata.angle_min = NEMA17.current_angle;
		
		
		
		
		
		
		std::cout << "c: " << NEMA17.count << " a:" << NEMA17.current_angle << std::endl;
		startTrigger = ros::Time::now();
		while (NEMA.current_angle > -M_PI)
		{
			if ((NEMA17.count % 100) == 0)
			{
				triggerOneShotECLIDAR(I2C_Handle_LIDAR);
			}
			else
			{
				triggerOneShotLIDAR(I2C_Handle_LIDAR);
			}
			pollLIDAR(I2C_Handle_LIDAR);
			range = readLIDAR(I2C_Handle_LIDAR);
			std::cout << "c: " << NEMA17.count << " a:" << NEMA17.current_angle << " r:" << range << std::endl;
			stepDRV8825(&NEMA17);
			LIDARdata.ranges.push_back(range);
			std::cout << "c: " << NEMA17.count << " a:" << NEMA17.current_angle << std::endl;
		}
		endTrigger = ros::Time::now();
		LIDARdata.header.stamp = startScan;
		LIDARdata.angle_max = NEMA17.current_angle + NEMA17.angle_increment;
		LIDARdata.scan_time = (float)((endTrigger - startScan).toSec());
		LIDARdata.time_increment = (float)((LIDARdata.scan_time)/(((float)NEMA17.count) + 1.0));
		LaserScan_pub.publish(LIDARdata);
		
		
		dirDRV8825(&NEMA17, DRV8825_FORWARD);
		gpioDelay(1000);
		NEMA17.current_angle = -M_PI;
		LIDARdata.ranges.clear();
		LIDARdata.angle_min = NEMA17.current_angle;
	}

	
   	gpioTerminate();


	return EXIT_SUCCESS;

}

