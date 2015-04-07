/*
 * odometryProcessor.h
 * Description : This is header file for odometryProcessor.h
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
#ifndef __ODOMPROCESSOR_H_INCLUDED__  // if this header is not included yet
#define __ODOMPROCESSOR_H_INCLUDED__  // then include this header

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// This is the classs to define odometry process required for pose graph SLAM 
class OdomProcess{
	/*
	 * This the class to process odometry. 
	 */ 
	private:
		// Ros node handles and subscriber initialization
		ros::NodeHandle nh_;
		ros::Subscriber sub_;

	public:
		// odometry roll, pitch and yaw
		double roll, pitch, yaw;
		double pose_x,pose_y;
		// subscriber callback function
		void callbk(nav_msgs::Odometry odomData);
		// subcriber function for ros
		void subr();	
	};
#endif
