// example_node.cpp
// This is an example node to test the working of existing packages writen. It is still in development phase

#include <iostream>
#include <ros/ros.h>
#include "odomProcessor.h"
#include "pointCloudProcessor.h"
#include "poseGraph.h"
#include <Eigen/Dense>
#include <fstream>
#include <cmath>


int main(int argc, char **argv)
{
	// Pose graph SLAM main Node. 

	/**************** Defining variables ************/	
	std::ofstream myfile;
	myfile.open ("test1.txt");
	int count ;
	bool first;
	float dist ;
	float prev_odom[2];
	Eigen::Matrix4f tr_mat;


	/****************Initializing Ros components*************/
	ros::init(argc,argv,"pose_graph_node");

	// Starting the odometry process to fetch the data.
	OdomProcess od_p;
	// Starting the pointcloud process, to get the cloud data. 
	PointCloudProcessor pcl_p;
	// Initializing the graph struture of the pose graph 
	PoseGraph pg;
	// Starting the subcribers for incoming data.
	od_p.subr();
	pcl_p.subr();		

	/****************** Initializing local variables ************/
	first = true;
	count = 0;
	dist = 0.0;


	/* while the process is going on do the stuff */
	while (ros::ok()){
	
		// Recieve the odometry data for current Iteration. what is the use ??.
		std::vector<double> odom_data;
		odom_data.push_back(od_p.pose_x);
		odom_data.push_back(od_p.pose_y);
		odom_data.push_back(od_p.yaw);		
		
		/* For the first cloud recieved initialize vertex ********/
		if((pcl_p.cloud_seq_loaded.size()==1) && first){
			std::cout<<"-->Initalizing pose graph"<< std::endl;
			std::vector<double> odom_init;
			odom_init.push_back(od_p.pose_x);
			odom_init.push_back(od_p.pose_y);
			odom_init.push_back(od_p.yaw);	
			prev_odom[0] = od_p.pose_x;
			prev_odom[1] = od_p.pose_y;
			// add the odometry data to the current vertex
			pg.addVertex(count, odom_init);
			first = false;
		}

		/* On recieving data , compute */
		if(pcl_p.cloud_seq_loaded.size()==2){
			float curr_odom[2];
			curr_odom[0] = od_p.pose_x;
			curr_odom[1] = od_p.pose_y;
			// calculate the distance
			dist = pow((curr_odom[0] - prev_odom[0]),2) + pow((curr_odom[1]-prev_odom[1]),2);
			dist = sqrt(dist);
			// if robot traverses a distance greater than 0.5 m then add the next node.
			if (dist>= 0.5){
				prev_odom[0] = curr_odom[0];
			 	prev_odom[1] = curr_odom[1];
				std::cout<<"Count is :"<< count<< std::endl;
				// estimate the transition matrix for the current postion wrt to previous position saved. 
				tr_mat = pcl_p.estTrans();
				//std::cout << pcl_p.calcICP() <<std::endl;
				count++;
				// add the data as vertex to graph and update the edge weight with transformation matrix.
				pg.addVertex(count,odom_data);
				pg.addEdgeToPrev(tr_mat);
				//std::cout << tr_mat << std::endl;
			}	
		}
		
		// To display the graph struture created.
		//pg.display();
		
		// While the ros is reciving data continiously subcribe to different topics
		ros::spinOnce();
	}
	
	// In order to generate file corresponding to the graph structure.
	
	/* First write vertex onto file */
	boost::tie(pg.vertexIt_, pg.vertexEnd_) = boost::vertices(pg.gr_);
	for(;pg.vertexIt_!=pg.vertexEnd_;++pg.vertexIt_)
	{
		myfile<<"Vertex ";
		myfile<<pg.gr_[*pg.vertexIt_].key;
		myfile<<" ";
		for (int i =0;i< 3;i++){
			myfile<<pg.gr_[*pg.vertexIt_].data[i];
			//std::cout<<pg.gr_[*pg.vertexIt_].data[i]<<std::endl;;
			myfile<<" ";
			}
		myfile<<"\n";
		
		std::cout<<std::endl;
		}

	/* Write edges to same file */	
	boost::tie(pg.edgeIt_,pg.edgeEnd_) = boost::edges(pg.gr_);
	for(;pg.edgeIt_!=pg.edgeEnd_;++pg.edgeIt_)
	{
		//myfile<<*pg.vertexIt_<;
		myfile<<"Edge ";
		Eigen::Matrix4f trfMat = pg.gr_[*pg.edgeIt_].transformation;
		for(int i = 0;i<16;i++){
			myfile<< *(trfMat.data()+i)<< " ";
		} 
		/* 
		for(size_t i =0;i<=trfMat.rows();i++ ){
			for (size_t j = 0; j<=trfMat.cols();j++){
				myfile<< *(trfMat.data()+i+j)<< " ";;	
			}
			
			}*/
		//myfile<<pg.gr_[*pg.edgeIt_].transformation;
		myfile<< "\n";
		
		}
	
	
	/*
	boost::tie(neighbourIt,neighbourEnd) = boost::adjacent_vertices(*vertexIt,g);
		for(;neighbourIt != neighbourEnd;++neighbourIt)
		std::cout<<*neighbourIt<<" ";
	*/
	
	// close file (Important)
	myfile.close();
	
	return 0;
}
h