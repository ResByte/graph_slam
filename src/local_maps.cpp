/* This contains local map generation function.
 * It reads point cloud sensor data and use explicit duration HMM to find out dynamic regions.
 * 
 */ 
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
#include <pcl/filters/random_sample.h>
#include <pcl/registration/transforms.h> 
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <Eigen/Dense>
#include <list>
#include <fstream>
using namespace std;
typedef pcl::PointXYZ PointT; //point type for octree structure
		//define octree point container
		typedef pcl::PointCloud<PointT> CloudT;
		typedef pcl::PointCloud<PointT>::Ptr CloudPtrT;	
		
class LocalMaps{
	private:
		//define parameters needed for this class
		
		
		//define ros parameters
		ros::NodeHandle nh;
		ros::Subscriber cloud;
		ros::Publisher pub;
		pcl::PCLPointCloud2::Ptr cloud_ptr;
		pcl::PCLPointCloud2 cloud_ ; 
		CloudT curr_pc;
		
	public:
		//define constructor
		LocalMaps();
		// read sensors data
		void cloudCallbk(sensor_msgs::PointCloud2 msg);
		void subCloud();
		CloudT randomSample(CloudT in_cld);		//filter cloud using voxel grid filter 
		//void filterCloud(CloudPtrT inCloud);
	};

CloudT LocalMaps::randomSample(CloudT in_cld){
	// Randomly sample 1000 pts from the cloud to calculate 2d rigit transform
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cld_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*in_cld_ptr = in_cld;
	pcl::RandomSample<pcl::PointXYZ> sample(true);
	sample.setInputCloud(in_cld_ptr);
	sample.setSample(SS);  // 1000 pts
	std::vector<int> out_idx;
	sample.filter(out_idx);
	pcl::PointCloud<pcl::PointXYZ> out_cld;
	sample.filter(out_cld);
	std::cout<<out_cld.size()<<std::endl;
	return out_cld;
}

// Cloud calback function for subscriber. Filtering of cloud to remove nan indices can be done here
void LocalMaps::cloudCallbk(sensor_msgs::PointCloud2 msg){
	//create empty vector to catch nan indices
	std::vector<int> nan_indices;
	//convert pointcloud from ros format to pcl format
	pcl_conversions::toPCL(msg,cloud_);
	//further convert point to given point type PointT
	pcl::fromPCLPointCloud2(cloud_,curr_pc);
	// finally remove nan indices from the point cloud
	pcl::removeNaNFromPointCloud(curr_pc,curr_pc,nan_indices);	
	}

// this is point cloud subscriber function
void LocalMaps::subCloud(){
	uint32_t queue_size = 1;
	cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,&LocalMaps::cloudCallbk, this);
	}
	

LocalMaps::LocalMaps(){
	//while the node is running repeatedly perform certain functions
	while(ros::ok()){
		
		this->subCloud();
	
		ros::spinOnce();
		
		}
}




int main(int argc, char **argv)
{	// initialize ros components
	ros::init(argc,argv,"local_maps");
	
	LocalMaps map;

	
	return 0;
}

