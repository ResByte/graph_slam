

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <deque>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/don.h>
#include <pcl/registration/registration.h>
#include <Eigen/Dense>
#include <boost/graph/graphviz.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>


std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq_loaded;
Eigen::Matrix4f tr_mat; 

void algoICP(pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc, pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc){
	std::cout << "2"<<std::endl;	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(first_pc);
	icp.setInputTarget(second_pc);

	//pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans_lls (new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZ, pcl::PointXYZ>);
	//icp.setTransformationEstimation (trans_lls);
	icp.setMaximumIterations(2);
	icp.setRANSACIterations(5);
	icp.setTransformationEpsilon (1e-6);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	tr_mat = icp.getFinalTransformation();
		
	//std::cout<<tr_mat<<std::endl;
			
}
/*
void algoFFPH(pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc, pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc){
	std::cout << "5"<<std::endl;	
	pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::NormalEstimation<pcl::PointXYZ,pcl::PointNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  	norm_est.setSearchMethod (tree);
  	norm_est.setKSearch (30);
  	norm_est.setInputCloud (first_pc);
  	norm_est.compute (*normals);
  	pcl::copyPointCloud (*first_pc, *normals);
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  	fpfh.setInputNormals(normals);
  	fpfh.setSearchMethod (tree);
  	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

  	// Use all neighbors in a sphere of radius 5cm
  	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  	fpfh.setRadiusSearch (0.05);

  	// Compute the features


  	fpfh.compute (*fpfhs);

	//std::cout<<tr_mat<<std::endl;
			
}


void filterCloud(pcl::pointcloud<pcl::PointXYZ>::Ptr  in_cloud){
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

}
*/
void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_0_ptr = cloud_in;
	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
	
  	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_0_ptr));
  

  	std::vector<int> inliers;
	//pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());	
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  	pcl::copyPointCloud<pcl::PointXYZ>(*cloud_0_ptr, inliers, *final);
  	
  	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_hull (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(final);     
	chull.setDimension(2);
    chull.reconstruct(*ground_hull);

    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    pcl::PointIndices::Ptr cloud_indices (new pcl::PointIndices);
	prism.setInputCloud (cloud_0_ptr);
	prism.setInputPlanarHull (ground_hull);
	prism.setHeightLimits (0.05, 5.0);
	prism.segment (*cloud_indices);

	pcl::ExtractIndices<pcl::PointXYZ> eifilter (true);
	eifilter.setInputCloud (cloud_0_ptr);
	eifilter.setIndices (cloud_indices);
	eifilter.filterDirectly (cloud_0_ptr);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> dy_sor;
	dy_sor.setInputCloud (cloud_0_ptr);
	dy_sor.setMeanK (20);
	dy_sor.setStddevMulThresh (0.5);
	dy_sor.filter (*cloud_0_ptr);

	pcl::PassThrough<pcl::PointXYZ> pass_2;
	pass_2.setInputCloud (cloud_0_ptr);
	pass_2.setFilterFieldName ("z");
	pass_2.setFilterLimits (0.0, 3.0);
	pass_2.filter (*cloud_0_ptr);

	pcl::VoxelGrid<pcl::PointXYZ> vxl_;
	vxl_.setInputCloud (cloud_0_ptr);
	vxl_.setLeafSize(0.05,0.05,0.05);
	vxl_.filter (*cloud_0_ptr);	

}	
void processCloud(const sensor_msgs::PointCloud2 msg)
{
	std::cout<<"Receiving cloud"<<std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr curr_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud;
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
	std::vector<int> nan_indices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr prev_pc (new pcl::PointCloud<pcl::PointXYZ>); 
		
	
	//********* Retirive and process raw pointcloud************
	pcl_conversions::toPCL(msg,cloud);
	pcl::PCLPointCloud2::Ptr cloud_ptr  (new pcl::PCLPointCloud2 ());
	*cloud_ptr = cloud;
	/*
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_ptr);
	sor.setLeafSize (0.05f, 0.05f, 0.05f);
	sor.filter (*cloud_ptr);
	*/
	pcl::fromPCLPointCloud2(cloud,pcl_pc);
	pcl::removeNaNFromPointCloud(pcl_pc,pcl_pc,nan_indices);
	*curr_pc =pcl_pc;
	filterCloud(curr_pc);
	
	cloud_seq_loaded.push_back(pcl_pc);

	
	if(cloud_seq_loaded.size()>2){
		cloud_seq_loaded.pop_front();
	}
	if(cloud_seq_loaded.size()==2)
	{
		std::cout << "1"<<std::endl;	
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc (new pcl::PointCloud<pcl::PointXYZ>);
		*first_pc = cloud_seq_loaded[0];
		*second_pc = cloud_seq_loaded[1];
		//algoFFPH(first_pc,second_pc);
		algoICP(first_pc,second_pc);	
		}
		
	 
	
	std::cout<<"finished"<<std::endl;
	std::cout<<std::endl;	
	}




int main(int argc, char **argv)
{	
	
	// Initialize ros nodes
	ros::init(argc, argv, "pose_graph");
	ros::NodeHandle nh;//create handle
	uint32_t queue_size = 1;
	
	/*
	 * Now, Point cloud is subscribed to from the kinect sensor, however this acquisition is slow and needs to be updated
	 */ 
	ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,processCloud);
	
	ros::spin();	
	return 0;
}

