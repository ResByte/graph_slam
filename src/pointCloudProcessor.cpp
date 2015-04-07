/*
 * pointCloudProcessor.cpp
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
//TODO : 

#include "pointCloudProcessor.h"
// constructor	


// Callback function for sensor_msg pointcloud	
void PointCloudProcessor::pclCallbk(sensor_msgs::PointCloud2 msg){
	//std::cout<< "subscribing to pcl" <<std::endl;
	std::cout<< "-->Starting point cloud Callback" <<std::endl;
	std::vector<int> nan_indices;
	pcl_conversions::toPCL(msg,cloud_);
	pcl::fromPCLPointCloud2(cloud_,curr_pc_);
	pcl::removeNaNFromPointCloud(curr_pc_,curr_pc_,nan_indices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_ptr = curr_pc_;
	//this->filterCloud(cloud_ptr);
	//TODO:Add loop closure
	//this->extractFeatures(cloud_ptr);
	cloud_seq_loaded.push_back(curr_pc_);
	if(cloud_seq_loaded.size()>2){
		cloud_seq_loaded.pop_front();
		//pose_seq_loaded.pop_front();
	}
	/*
	for (size_t i = 0; i < curr_pc_.size();i++){
		std::cout << curr_pc_.points[i]<<std::endl;
	}*/
}


void PointCloudProcessor::filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
	std::cout << "-->Filtering point cloud" << std::endl;
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

	pcl::PassThrough<pcl::PointXYZ> pass_1;
	pass_1.setInputCloud (cloud_0_ptr);
	pass_1.setFilterFieldName ("z");
	pass_1.setFilterLimits (0.0, 3.0);
	pass_1.filter (*cloud_0_ptr);

}


// USing ICP from PCL 

Eigen::Matrix4f PointCloudProcessor::calcICP(){
	std::cout<<"-->Calculating ICP and fetching transformation"<< std::endl;
	bool flag = true;

	pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc (new pcl::PointCloud<pcl::PointXYZ>);
	*first_pc = cloud_seq_loaded[0];
	*second_pc = cloud_seq_loaded[1];
	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//pcl::Registration<pcl::PointXYZ,pcl::PointXYZ,float tmp>::TransformationEstimationPointToPlaneLLS < pcl::PointXYZ,pcl::PointXYZ>::Ptr trans_lls (new pcl::Registration<pcl::PointXYZ,pcl::PointXYZ,float tmp>::TransformationEstimationPointToPlaneLLS<pcl::PointXYZ, pcl::PointXYZ>);
	icp.setInputSource(first_pc);
	icp.setInputTarget(second_pc);
	//icp.setTransformationEstimation (trans_lls);
	icp.setMaximumIterations (2);
	icp.setRANSACIterations(5);
  	//icp.setTransformationEpsilon (1e-8);
  	//icp.setMaxCorrespondenceDistance (0.5);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	tr_mat_ = icp.getFinalTransformation();
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	

	return tr_mat_;
	}
void PointCloudProcessor::subr(){
	std::cout<< "-->Subscribing to Point cloud" << std::endl;
	uint32_t queue_size = 1;
	sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,&PointCloudProcessor::pclCallbk, this);
	
	}

void PointCloudProcessor::extractFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
	//TODO: extract feature points
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_ptr = cloud_in;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  	ne.setSearchMethod (tree);

  	// Output datasets
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  	// Use all neighbors in a sphere of radius 3cm
  	ne.setRadiusSearch (0.03);
  	// Compute the features
  	ne.compute (*cloud_normals);
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud_ptr);
  	fpfh.setInputNormals (cloud_normals);
  	fpfh.setSearchMethod (tree);
  	// Output datasets
  	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
  	// Use all neighbors in a sphere of radius 5cm
  	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  	fpfh.setRadiusSearch (0.05);
  	// Compute the features
  	fpfh.compute (*fpfhs);
  	std::cout<<fpfhs->points.size()<<std::endl;	
}
pcl::PointCloud<pcl::PointXYZ> PointCloudProcessor::randomSample(pcl::PointCloud<pcl::PointXYZ> in_cld){
	// Randomly sample 1000 pts from the cloud to calculate 2d rigit transform
	std::cout<< "Randomly samping with size :";
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cld_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*in_cld_ptr = in_cld;
	pcl::RandomSample<pcl::PointXYZ> sample(true);
	sample.setInputCloud(in_cld_ptr);
	sample.setSample(5000);  // 1000 pts
	std::vector<int> out_idx;
	sample.filter(out_idx);
	pcl::PointCloud<pcl::PointXYZ> out_cld;
	sample.filter(out_cld);
	std::cout<<out_cld.size()<<std::endl;
	return out_cld;
}	
Eigen::Matrix4f PointCloudProcessor::estTrans(){
	/* Error that the size of both the cloud should be same
		Solution: randomly sample same number of points and get transform for those
		
	*/
	std::cout<< "--> estimating transform 2D \n";
	pcl::registration::TransformationEstimation2D< pcl::PointXYZ, pcl::PointXYZ, float >  ddTr; 	
	pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc (new pcl::PointCloud<pcl::PointXYZ>);
	*first_pc = this->randomSample(cloud_seq_loaded[0]);
	*second_pc = this->randomSample(cloud_seq_loaded[1]);
	 
	Eigen::Matrix4f result;
	ddTr.estimateRigidTransformation(*first_pc,*second_pc,result);
	std::cout<<result<<std::endl;
	return result;
}

/*			
int main(int argc, char **argv)
{	ros::init(argc, argv, "PointCloudrocessor");
	PointCloudProcessor cld_process;

	cld_process.subr();
	ros::spin();
	
	
	return 0;
}*/

