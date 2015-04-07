/*
 * pointCloudProcessor.h
 * Description : Header file for pointCloudProcessor.cpp
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
#ifndef __POINTCLOUDPROCESSOR_H_INCLUDED__  // if this header is not included yet
#define __POINTCLOUDPROCESSOR_H_INCLUDED__  // then include this header

#include <iostream>
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <boost/make_shared.hpp> 
#include <pcl/features/normal_3d.h>
 #include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
 #include <pcl/filters/project_inliers.h>
 #include <pcl/ModelCoefficients.h>
 #include <pcl/surface/convex_hull.h>
 #include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/fpfh.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <Eigen/Dense>
#include <deque>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
 #include <pcl/registration/transformation_estimation_2D.h>

 
class PointCloudProcessor{
	
		
	public:
		std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq_loaded; 
		pcl::PCLPointCloud2::Ptr cloud_ptr_ ;
		pcl::PCLPointCloud2 cloud_ ; 
		// initializing only pointer without memory allocations results in error
		
		//*cloud_ptr_ = cloud_;
		
		//pcl::PointCloud<pcl::PointXYZ>::Ptr curr_pc_ptr_ ;
		pcl::PointCloud<pcl::PointXYZ> curr_pc_;
		//*curr_pc_ptr_ = curr_pc_;
		
		ros::NodeHandle nh_;
		ros::Subscriber sub_;
		ros::Publisher pub_;
		Eigen::Matrix4f tr_mat_;
		Eigen::Matrix4f prev_tr_mat_;
		pcl::visualization::PCLVisualizer *p;
		class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
		{
  			using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
			public:
  				MyPointRepresentation ()
  				{
    				// Define the number of dimensions
    				nr_dimensions_ = 4;
  				}

  				// Override the copyToFloatArray method to define our feature vector
  				virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
  				{
    				// < x, y, z, curvature >
    				out[0] = p.x;
    				out[1] = p.y;
    				out[2] = p.z;
    				out[3] = p.curvature;
  				}
		};
		//constructor
		void subr();
		void pclCallbk(sensor_msgs::PointCloud2 msg);
		//void fetchCloud();
		void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
		//Calculate Iterative closest point matching between the 2 point clouds
		Eigen::Matrix4f calcICP();
		void extractFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
		Eigen::Matrix4f estTrans();	
		pcl::PointCloud<pcl::PointXYZ> randomSample(pcl::PointCloud<pcl::PointXYZ> in_cld);
	};
	
#endif	
