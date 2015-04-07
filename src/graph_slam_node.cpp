/*
 * node.cpp
 * 
 * Copyright 2014 abhinav <abhinav@abhinav-VirtualBox>
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

#include "graph_slam_node.h"
#include "poseGraph.h"

typedef std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq;
typedef std::deque<std::list<double> > pose_seq;
Eigen::Matrix4f tr_mat;
double roll, pitch, yaw;
double pose_x,pose_y;
cloud_seq cloud_seq_loaded; 
pose_seq pose_seq_loaded;
const float res = 0.1;
pcl::PointCloud<pcl::PointXYZ> static_pc;
int count = 0;
PoseGraph pg;
octomap::OcTree tree(0.01);
Eigen::Vector4f curr_origin(0.0f,0.0f,0.0f,1.0f);

void addCloud(pcl::PointCloud<pcl::PointXYZ> cld ){
	octomap::Pointcloud pc;
	octomap::point3d cld_origin(curr_origin(0),curr_origin(1),curr_origin(2));
	for(int i = 0;i<cld.points.size();i++){
		pc.push_back((float) cld.points[i].x,(float) cld.points[i].y,(float) cld.points[i].z);
	}
	tree.insertPointCloud(pc,cld_origin, -1, false,false);
	}


void createGraph(const Eigen::Matrix4f tr_msg){
	pg.addVertex(count);
	pg.addEdgeToPrev(tr_msg);
	count++;
	}

void getOdom(nav_msgs::Odometry data){
	/*
	 *This function gets the odometry msg from the robot 
	 * and using TF gives the angle in euclidean form. 
	 * 
	 */
	
	//std::cout<<"Receiving odometry"<<std::endl;
	//std::cout<< data.pose.pose.orientation<<std::endl;
	tf::Quaternion q(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

	pose_x = data.pose.pose.position.x;
	pose_y = data.pose.pose.position.y;
	m.getRPY(roll, pitch, yaw);

	//std::cout << "x: " << pose_x << ", y : " << pose_y << ", Yaw: " << yaw << std::endl;
	//std::cout<< v[1];	
	}
		
void processCloud(const sensor_msgs::PointCloud2 msg)
{
	std::cout<<"Receiving cloud"<<std::endl;
	/*std::cout << "x: " << pose_x << ", y : " << pose_y << ", Yaw: " << yaw << std::endl;
	double myPoses[] = {pose_x,pose_y,yaw};
	std::list<double> curr_pose (myPoses, myPoses + sizeof(myPoses) / sizeof(double) );
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr curr_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud;
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
	std::vector<int> nan_indices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr prev_pc (new pcl::PointCloud<pcl::PointXYZ>); 
		
	
	//********* Retirive and process raw pointcloud************
	pcl_conversions::toPCL(msg,cloud);
	pcl::PCLPointCloud2::Ptr cloud_ptr  (new pcl::PCLPointCloud2 ());
	*cloud_ptr = cloud;
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_ptr);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_ptr);
	
	pcl::fromPCLPointCloud2(cloud,pcl_pc);
	pcl::removeNaNFromPointCloud(pcl_pc,pcl_pc,nan_indices);
	*curr_pc =pcl_pc;
	
	//***************Filter point cloud to detect nearby changes only *****************
	/*	pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (curr_pc);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0, 3.0);
		pass.filter (*curr_pc);
	*/	 
		
		
		/*
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> dy_sor;
		dy_sor.setInputCloud (curr_pc);
		dy_sor.setMeanK (20);
		dy_sor.setStddevMulThresh (1.0);
		dy_sor.filter (*curr_pc);
		*/
	cloud_seq_loaded.push_back(pcl_pc);
	//pose_seq_loaded.push_back(curr_pose);
	std::cout<<pose_seq_loaded.size()<<std::endl;
	
	if(cloud_seq_loaded.size()>2){
		cloud_seq_loaded.pop_front();
		//pose_seq_loaded.pop_front();
	
	}
	if(cloud_seq_loaded.size()==1){
		//Add first vertex ;
		/*Vertex v;
		v = boost::add_vertex(g);
		g[v].key = count;
		g[v].data  = pose_seq_loaded[0];
		count++;
		*/}
	if(cloud_seq_loaded.size()==2)
	{
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc (new pcl::PointCloud<pcl::PointXYZ>);
		*first_pc = cloud_seq_loaded[0];
		*second_pc = cloud_seq_loaded[1];
		icp.setInputSource(first_pc);
		icp.setInputTarget(second_pc);
		pcl::PointCloud<pcl::PointXYZ> Final;
		icp.align(Final);
		//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		//icp.getFitnessScore() << std::endl;
		//std::cout << icp.getFinalTransformation() << std::endl;
		tr_mat = icp.getFinalTransformation();
		
		//std::cout<<tr_mat<<std::endl;
		//createGraph(tr_mat);
		std::cout<<std::endl;
		//boost::write_graphviz(std::cout, g);
		//std::cout<<std::endl;
		curr_origin = tr_mat*curr_origin;
		//std::cout<<curr_origin<<std::endl;
		addCloud(cloud_seq_loaded[1]);
		/*
		ros::NodeHandle k;
		ros::Publisher pub = k.advertise<pcl::PointCloud<pcl::PointXYZ> >("dynamicPoints",2);
		pub.publish(cloud_seq_loaded[1]);
		//ros::Time time = ros::Time::now();
		//Wait a duration of one second.
		//ros::Duration d = ros::Duration(1.5, 0);
		//d.sleep();
		std::cout<<"published points"<<std::endl;
		ros::spinOnce();
		*/
		if (count ==10){
			tree.writeBinary("simple_tree.bt");
		
		}
		else{
			std::cout<<count<<std::endl;
			count++;
		}	
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
	
	// Add starting vertex 
	

	pg.addVertex(count);
	count++;
	// First the odometry value from the turtlebot is subscribed
	 
	ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry>("/odom",queue_size,getOdom);
	//ros::Subscriber sub_pose = nh.subscribe<nav_msgs::Odometry>("/pose",queue_size,getOdom);
	
	/*
	 * Now, Point cloud is subscribed to from the kinect sensor, however this acquisition is slow and needs to be updated
	 */ 
	ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,processCloud);
	
	ros::spin();	
	return 0;
}

