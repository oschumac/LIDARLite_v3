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

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

int32_t main(int argc, char *argv[])
{
	
	//Setup ROS
	ros::init(argc, argv, "LIDARLite_v3");
	ros::NodeHandle node;
	ros::Publisher LaserScan_pub = node.advertise<sensor_msgs::LaserScan>("scan", 500);
	ros::Publisher Cloud_pub = node.advertise<sensor_msgs::PointCloud2>("cloud_in", 500);
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
	if (I2C_Handle_LIDAR < 0)
	{
		#ifdef DEBUG
		std::cout << "Failed to init LIDAR" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
	

	//Stepper motor
	DRV8825pin NEMA17;
	bool isValid = initDRV8825(&NEMA17, 17,27, 22,11,9,10);
	if (isValid == false)
	{
		#ifdef DEBUG
		std::cout << "Failed to init DRV8825" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
	setStepMode(&NEMA17,4);
	dirDRV8825(&NEMA17, DRV8825_FORWARD);


	//Setup LaserScan data
	sensor_msgs::LaserScan LIDARdata;
	LIDARdata.header.frame_id = "laser";
	LIDARdata.range_min = LIDAR_RANGE_MIN;
	LIDARdata.range_max = LIDAR_RANGE_MAX;
	LIDARdata.angle_increment = NEMA17.angle_increment;
	LIDARdata.angle_min = NEMA17.current_angle;
	LIDARdata.angle_max = M_PI - NEMA17.angle_increment;
	float stepsPerCycle = (float)NEMA17.stepsPerCycle;
	LIDARdata.ranges.resize(NEMA17.stepsPerCycle);
	float rangeArray[NEMA17.stepsPerCycle];
	int32_t i;



	//Point cloud data
	sensor_msgs::PointCloud2 Pointdata;
	Pointdata.header.frame_id = "laser";
	Pointdata.width  = NEMA17.stepsPerCycle;
	Pointdata.height = 1;
	Pointdata.fields.resize(3);
	
	Pointdata.fields[0].name = "x";
	Pointdata.fields[1].name = "y";
	Pointdata.fields[2].name = "z";

	uint32_t offset = 0;
	size_t index;
	for (index = 0; index < Pointdata.fields.size(); ++index, offset += 4)
	{
		Pointdata.fields[index].count    = 1;
		Pointdata.fields[index].offset   = offset;
		Pointdata.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
	}
	size_t x_data_offset = Pointdata.fields[0].offset ;
	size_t y_data_offset = Pointdata.fields[1].offset ;

	Pointdata.point_step = offset;
	Pointdata.row_step = Pointdata.point_step * Pointdata.width;
	Pointdata.data.resize( Pointdata.row_step);
	std::fill(Pointdata.data.begin(), Pointdata.data.end(), 0);
	Pointdata.is_bigendian = false;
	Pointdata.is_dense = true;
	float pos_x,pos_y,theta;


	std::cout << "step per cycle: " << NEMA17.stepsPerCycle << std::endl;
	float range = 0.0;
	ros::Time startScan,endScan;
	while (ros::ok())
	{
		//std::cout << "c: " << NEMA17.count << " a:" << NEMA17.current_angle << std::endl;
		startScan = ros::Time::now();
		while (NEMA17.current_angle < M_PI)
		{
			//Start LIDAR
			if ((NEMA17.count % 100) == 0)
			{
				triggerOneShotECLIDAR(I2C_Handle_LIDAR);
			}
			else
			{
				triggerOneShotLIDAR(I2C_Handle_LIDAR);
			}
			//Wait for LIDAR
			pollLIDAR(I2C_Handle_LIDAR);
			//Get LIDAR range
			range = readLIDAR(I2C_Handle_LIDAR);
			//std::cout << "c: " << NEMA17.count << " a:" << NEMA17.current_angle << " r:" << range << std::endl;
			//Rotate
			stepDRV8825(&NEMA17);
			//Save data
			rangeArray[NEMA17.count-1]  = range;
			//LIDARdata.ranges.push_back(range);
			//std::cout << "c: " << NEMA17.count << " a:" << NEMA17.current_angle << std::endl;
		}
		endScan = ros::Time::now();
		//Calculate data
		LIDARdata.header.stamp = startScan;
		LIDARdata.scan_time = (float)((endScan - startScan).toSec());
		LIDARdata.time_increment = (float)((LIDARdata.scan_time)/(stepsPerCycle));
		for (i=0;i<NEMA17.stepsPerCycle;++i)
		{
			LIDARdata.ranges[i] = rangeArray[i];
	
			theta = (i*NEMA17.angle_increment) - M_PI;
			pos_x = (float)((rangeArray[i])*cos(theta));
			pos_y = (float)((rangeArray[i])*sin(theta));
			memcpy(&Pointdata.data[i * Pointdata.point_step + x_data_offset], &pos_x, sizeof(float));
			memcpy(&Pointdata.data[i * Pointdata.point_step + y_data_offset], &pos_y, sizeof(float));
		}
		LaserScan_pub.publish(LIDARdata);
		Cloud_pub.publish(Pointdata);
		
		NEMA17.current_angle = -M_PI;
		NEMA17.count = 0;
	}

	disableDRV8825(&NEMA17);	
   	gpioTerminate();


	return EXIT_SUCCESS;

}

