/*
 * odometryProcessor.cpp
 * Description : This is the class to process the data from odometry of the robot.
 * 
 * Copyright 2014 Shibata-Lab <shibata-lab@shibatalab-X500H>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */
#include "odomProcessor.h"

// This is the subcriber callback function that recieves nav msg odometry data and calculate yaw with x, y poses. Assuming only 2D motion.
void OdomProcess::callbk(nav_msgs::Odometry odomData){
	//std::cout << odomData << std::endl;
	std::cout<< "-->Updating odometry value" <<std::endl;
	tf::Quaternion q(odomData.pose.pose.orientation.x, odomData.pose.pose.orientation.y, odomData.pose.pose.orientation.z, odomData.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	pose_x = odomData.pose.pose.position.x;
	pose_y = odomData.pose.pose.position.y;
	m.getRPY(roll, pitch, yaw);
	//std::cout << pose_x << std::endl;
	}

// This is the subcriber function.
void OdomProcess::subr(){
	std::cout<< "-->Subscribing to Odometry" <<std::endl;
	uint32_t queue_size = 1;
	sub_ = nh_.subscribe<nav_msgs::Odometry>("/pose",queue_size,&OdomProcess::callbk,this);
	}	


/*
// Test
int main(int argc, char **argv)
{	ros::init(argc, argv, "PointCloudrocessor");
	OdomProcess op;
	op.subr();
	return 0;
}*/

