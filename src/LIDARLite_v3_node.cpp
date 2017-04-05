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
#include <string.h>

#include <ros/ros.h>

#define DEBUG 1

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include "include/LIDARLite_v3/LIDARLite_v3.h"
#include "include/LIDARLite_v3/DRV8825.h"

typedef struct LIDARdata
{
	ros::Publisher pub;
	sensor_msgs::PointCloud2 LIDARdata;
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
	ros::Publisher LIDAR_pub = node.advertise<sensor_msgs::PointCloud2>("cloud", 500);
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
	LIDAR1.LIDARdata.header.frame_id = "base_link";
	LIDAR1.LIDARdata.width  = 800;
	LIDAR1.LIDARdata.height = 1;
	LIDAR1.LIDARdata.fields.resize(3);
	

	LIDAR1.LIDARdata.fields[0].name = "x";
	LIDAR1.LIDARdata.fields[1].name = "y";
	LIDAR1.LIDARdata.fields[2].name = "z";

	uint32_t offset = 0;
	size_t index;
	for (index = 0; index < LIDAR1.LIDARdata.fields.size(); ++index, offset += 4)
	{
		LIDAR1.LIDARdata.fields[index].count    = LIDAR1.LIDARdata.width;
		LIDAR1.LIDARdata.fields[index].offset   = offset;
		LIDAR1.LIDARdata.fields[index].datatype = sensor_msgs::PointField::FLOAT32;
	}
	size_t x_data_offset = LIDAR1.LIDARdata.fields[0].offset ;
	size_t y_data_offset = LIDAR1.LIDARdata.fields[1].offset ;

	LIDAR1.LIDARdata.point_step = offset;
	LIDAR1.LIDARdata.row_step = LIDAR1.LIDARdata.point_step * LIDAR1.LIDARdata.width;
	LIDAR1.LIDARdata.data.resize( LIDAR1.LIDARdata.row_step);
	std::fill(LIDAR1.LIDARdata.data.begin(), LIDAR1.LIDARdata.data.end(), 0);
	LIDAR1.LIDARdata.is_bigendian = false;
	LIDAR1.LIDARdata.is_dense = true;
	
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

	double radius,theta;
	float pos_x,pos_y;
	while (ros::ok())
	{
		index = 0;
		LIDAR1.LIDARdata.header.stamp = ros::Time::now();
		while (NEMA17.count < 800)
		{
			if ((NEMA17.count % 100) == 0)
			{
				triggerOneShotECLIDAR(LIDAR1.I2C_Handle_LIDAR);
			}
			else
			{
				triggerOneShotLIDAR(LIDAR1.I2C_Handle_LIDAR);
			}
			pollLIDAR(LIDAR1.I2C_Handle_LIDAR);
		 
			radius = readLIDAR(LIDAR1.I2C_Handle_LIDAR);
			
			stepDRV8825(&NEMA17);
			theta = ((double)NEMA17.count)*((2.0*M_PI)/800.0);
			pos_x = (float)(radius*cos(theta));
			pos_y = (float)(radius*sin(theta));
			memcpy(&LIDAR1.LIDARdata.data[index * LIDAR1.LIDARdata.point_step + x_data_offset], &pos_x, sizeof(float));
			memcpy(&LIDAR1.LIDARdata.data[index * LIDAR1.LIDARdata.point_step + y_data_offset], &pos_y, sizeof(float));
			++index;
		}


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

	
   	gpioTerminate();


	return EXIT_SUCCESS;

}

