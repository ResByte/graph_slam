/*
 * node.cpp
 * 
 * Copyright 2015 Shibata-Lab <shibata-lab@shibatalab-X500H>
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
 * NOTE: This is written according to ROS c++ guidelines
 */


#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
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
#include <deque>
#include <fstream>
#include <pcl/registration/gicp.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graphviz.hpp>
#include <math.h>
using namespace std;
#define PI 3.14159265

class Node{
	public:
		float threshold_distance; 
		ros::NodeHandle nh;
		ros::Subscriber odom;
		ros::Subscriber cloud;
		
		double pose_x,pose_y,roll,pitch,yaw;
		pcl::PCLPointCloud2::Ptr cloud_ptr;
		pcl::PCLPointCloud2 cloud_ ; 
		pcl::PointCloud<pcl::PointXYZ> curr_pc;
		Eigen::Matrix4f tr_mat;
		Eigen::Matrix4f prev_tr_mat;
		
		std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq_loaded;
		
		/* properties for vertex of the graph */ 
		struct pose_{
			int key;
			std::vector<double>  data;
			pcl::PointCloud<pcl::PointXYZ> cld_data;
		};
		
		/* Custom edge properties as constraints */
		struct constraints_{
			int src;
			int obs;
			Eigen::Matrix4f transformation;
			double score;
		};
		
		/* store odometry values */
		struct odometry{
			double pose_x;
			double pose_y;
			double yaw;
		};
		
		/* Graph type and definitions */
		typedef boost::adjacency_list<boost::listS,boost::vecS, boost::undirectedS,pose_, constraints_> Graph;
		typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
		typedef boost::graph_traits<Graph>::edge_descriptor Edge;
		Graph gr;
		Graph::vertex_iterator vertex_It,vertex_End,vertex_i,vertex_e;
		Graph::edge_iterator edge_It,edge_End;
		
		
		/* main node process */ 
		Node(); //constructor
		/* Get Odometry data */
		void odomSub();
		
		/* procees odometry data to get 2D pose */
		void odomCallbk(nav_msgs::Odometry odom_data);
		
		/* Get Point Cloud data */
		void cloudSub();
		
		/* process point cloud data */ 
		void cloudCallbk(sensor_msgs::PointCloud2 msg);
		
		/* calculate Transform between two point clouds */ 
		Eigen::Matrix4f estTrans(pcl::PointCloud<pcl::PointXYZ> first,pcl::PointCloud<pcl::PointXYZ> second);	
		pcl::PointCloud<pcl::PointXYZ> randomSample(pcl::PointCloud<pcl::PointXYZ> in_cld);
	
		void detectLoopClosure(Vertex curr_v);
		/* calculate distance travelled */ 
		double calculateDist(vector<double> curr_odom,vector<double> prev_odom);
		
		/* Add vertex to graph with given data */ 
		void addVertex(int v,std::vector<double> odom);
		
		/* Add edge to the last added vertex */ 
		void addEdge(Eigen::Matrix4f tr_msg,double tr_score);
		
		/* Initialize graph */ 
		void initGraph(int v,std::vector<double> odom);
	};
	
/* Callback function for odom subscriber */	
void Node::odomCallbk(nav_msgs::Odometry odom_data){
	/* Calculate quaternion to get roll, pitch , yaw */
	tf::Quaternion q(odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	pose_x = odom_data.pose.pose.position.x;
	pose_y = odom_data.pose.pose.position.y;
	m.getRPY(roll, pitch, yaw);
	}
	
/* Odometry subscriber function */
void Node::odomSub(){
	uint32_t queue_size = 1;
	odom = nh.subscribe<nav_msgs::Odometry>("/odom",queue_size,&Node::odomCallbk,this);
	}	

pcl::PointCloud<pcl::PointXYZ> Node::randomSample(pcl::PointCloud<pcl::PointXYZ> in_cld){
	// Randomly sample 1000 pts from the cloud to calculate 2d rigit transform
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cld_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*in_cld_ptr = in_cld;
	pcl::RandomSample<pcl::PointXYZ> sample(true);
	sample.setInputCloud(in_cld_ptr);
	sample.setSample(2500);  // 1000 pts
	std::vector<int> out_idx;
	sample.filter(out_idx);
	pcl::PointCloud<pcl::PointXYZ> out_cld;
	sample.filter(out_cld);
	std::cout<<out_cld.size()<<std::endl;
	return out_cld;
}

Eigen::Matrix4f Node::estTrans(pcl::PointCloud<pcl::PointXYZ> first,pcl::PointCloud<pcl::PointXYZ> second){
	/* Error that the size of both the cloud should be same
		Solution: randomly sample same number of points and get transform for those
			Estimating 2D transform between cloud points
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr first_pc (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr second_pc (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ> first_pc;
	//pcl::PointCloud<pcl::PointXYZ> second_pc;
	pcl::PointCloud<pcl::PointXYZ> final;
	
	*first_pc = this->randomSample(first);
	*second_pc = this->randomSample(second);
	
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
	gicp.setMaximumIterations(5); // no. of Iterations for optimization
    	gicp.setRANSACIterations(5); // no. of iterations for Ransac part.
	
    gicp.setInputCloud(first_pc);
    gicp.setInputTarget(second_pc);

    Eigen::Matrix4f result;
    
    gicp.align(final);
	result = gicp.getFinalTransformation();
	cout << gicp.getFitnessScore() << endl;
	cout << gicp.getFinalTransformation() << endl;
	//pcl::registration::TransformationEstimation2D< pcl::PointXYZ, pcl::PointXYZ, float >  ddTr; 	 
	
	//ddTr.estimateRigidTransformation(*first_pc,*second_pc,result);
	//std::cout<<result<<std::endl;
	return result;
}

/* callback function for cloud msg */	
void Node::cloudCallbk(sensor_msgs::PointCloud2 msg){
	cout<<"cloud call back"<<endl;
	std::vector<int> nan_indices;
	pcl_conversions::toPCL(msg,cloud_);
	pcl::fromPCLPointCloud2(cloud_,curr_pc);
	pcl::removeNaNFromPointCloud(curr_pc,curr_pc,nan_indices);	
	}

/* point cloud subscriber */
void Node::cloudSub(){
	cout<<"cld Subr"<<endl;
	uint32_t queue_size = 1;
	cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",queue_size,&Node::cloudCallbk, this);
	cout<<"data fetched"<<endl;
	}

/* calculate distance travelled */ 
double Node::calculateDist(vector<double> curr_odom,vector<double> prev_odom){
	double dist;	
	dist = pow((curr_odom[0] - prev_odom[0]),2) + pow((curr_odom[1]-prev_odom[1]),2);
	dist = sqrt(dist);
	return dist;
	}
		
/* Add vertex to graph with given data */ 
void Node::addVertex(int v,std::vector<double> odom){
	Vertex v1;
	v1 = boost::add_vertex(gr);
	gr[v1].key = v ;
	gr[v1].data = odom;
	gr[v1].cld_data = curr_pc;
	return;
	}
		
/* Add edge to the last added vertex */ 
void Node::addEdge(Eigen::Matrix4f tr_msg,double tr_score){
	//Graph::vertex_iterator vertex_It,vertex_End;
	boost::tie(vertex_It, vertex_End) = boost::vertices(gr);
	//std::cout<< "-->Vertex End is :"<<*vertexEnd<<std::endl;
	Edge e1; 
	e1 = (boost::add_edge(*(vertex_End-2),(*vertex_End-1),gr)).first;
	gr[e1].transformation = tr_msg;
	gr[e1].src = gr[*vertex_End-2].key;
	gr[e1].obs = gr[*vertex_End-1].key;
	gr[e1].score = tr_score;
	}
		
/* Initialize graph */ 
void Node::initGraph(int v,std::vector<double> odom){
	Vertex v1;
	v1 = boost::add_vertex(gr);
	gr[v1].key = v ;
	gr[v1].data = odom;
	gr[v1].cld_data = curr_pc;
	}


void Node::detectLoopClosure(Vertex curr_v){
	/* If the odom value is nearest to a vertex then add edge.
	 * Loop closure is detected by comparing odom value,  if they are less than threshold. 
	 *  */
	// since graph is not big , iterating over all of the veritces .Otherwise various searching methods can be implemented 
	boost::tie(vertex_It, vertex_End) = boost::vertices(gr);
	double dist;
	for(;vertex_It!=vertex_End;++vertex_It){
		// it is not equal to curr vertex and there is no edge already present
		dist = this->calculateDist(gr[curr_v].data,gr[*vertex_It].data);
		cout<<dist<<endl;
		if (!boost::edge(curr_v,(*vertex_It),gr).second && dist < 0.15 && dist != 0.0){
			cout<<"Loop Closure detected"<<endl;
			Eigen::Matrix4f transform  = this->estTrans(gr[*vertex_It].cld_data, gr[curr_v].cld_data);
			Edge e;
			e = (boost::add_edge(curr_v,*(vertex_It),gr)).first;
			gr[e].transformation = transform;
			cout <<gr[curr_v].key<<" "<<gr[*vertex_It].key<<endl;
			gr[e].src = gr[curr_v].key;
			gr[e].obs = gr[*vertex_It].key; // What is vertex_it pointing to 
			gr[e].score = 0.0;
			boost::write_graphviz(cout, gr);
			}
		
	}	
	
	}	
		
/* main node process as well as constructor*/ 
Node::Node()
{
	pcl::PointCloud<pcl::PointXYZ> prev_cld; // one step buffer for clouds comparision
	int count = 1; // counter for key of the vertex
	threshold_distance=0.15; // thesholding for the distance travelled
	
	
	/* Subscribe to odometry */ 
	while(ros::ok())
	{
		cout<<"Subscribe to odometry"<<endl;
		this->odomSub(); // data-> pose_x,pose_y,roll, pitch, yaw
	
		/* Subscribe to point cloud */
		cout<<"Subscribe to point cloud"<<endl;
		this->cloudSub(); //data-> curr_pc
		
		if(curr_pc.size() > 0)
			break;
			
		ros::spinOnce();
	}
	//cloud_seq_loaded.push_back(curr_pc);
	
	/* Initialize graph with first vertex as the starting point */
	std::vector<double> curr_odom;
	curr_odom.push_back(pose_x);
	curr_odom.push_back(pose_y);
	curr_odom.push_back(yaw);
	//cout<<"Initialize graph with first vertex as the starting point"<<endl;
	this->initGraph(0,curr_odom); //data-> gr graph
	
	/* Save the first cloud */ 
	prev_cld = curr_pc;
	/* Whle the process is going on loop it */
	//cout<<"Whle the process is going on loop it"<<endl;
	
	while(ros::ok())
	{
		/*subscribe to odom */ 
		//cout<<"subscribe to odom"<<endl;
		this->odomSub();
		
		/* Subscribe to point cloud */ 
		//cout<<"Subscribe to point cloud"<<endl;
		this->cloudSub();
		
		cout << "Current Odom: " << pose_x << " " << pose_y << endl;
		cout << "Cloud Size: " << curr_pc.size() << endl;
		
		/* Calculate distance */
		cout<<"distance"<<endl;
		double distance;
		std::vector<double> now_odom;
		now_odom.push_back(pose_x);
		now_odom.push_back(pose_y);
		now_odom.push_back(yaw);
		
		//cout<<"calculating prev odom value"<<endl;
		boost::tie(vertex_It,vertex_End) = boost::vertices(gr);
		std::vector<double> prev_odom;
		prev_odom = gr[*vertex_End-1].data;
		//cout<<"Calculate distance"<<endl;
		distance = this->calculateDist(now_odom, prev_odom);
		//cout<<distance<<endl;
		
		/* if distance is greater than threshold */
		if(distance >= threshold_distance && curr_pc.size() > 0)
		{
			/*  add vertex to graph  */
			//cout<<"-->add vertex to graph"<<endl;
			
			this->addVertex(count,now_odom);
			/* calculate transform */
			//cout<<"-->calculate transform"<<endl;
			cout << curr_pc.size() << " " << prev_cld.size() << endl;
			tr_mat =this->estTrans(curr_pc,prev_cld);
			/* Add edge weights between current and previous */
			//cout<< "-->Add edge weights between current and previous"<<endl;
			
			
			/* evaluate the validity of edge transforms */
			/*
			pcl::registration::TransformationValidationEuclidean<pcl::PointXYZ, pcl::PointXYZ> tve;
			tve.setMaxRange (0.1); // 1cm
			pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
			*source = prev_cld;
			pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
			*target = curr_pc;
			double score = tve.validateTransformation (source, target, tr_mat);
			cout<<score<<"<--Tr score"<<endl;
			*/
			
			boost::tie(vertex_i, vertex_e) = boost::vertices(gr);
			this->addEdge(tr_mat,0.000);
			this->detectLoopClosure(*(vertex_e-1));
			prev_cld = curr_pc;
			count++;
		}
		
		ros::spinOnce();
	}
		
		/* write data onto a file for later optimization */
		std::ofstream myfile,file;
		myfile.open ("example.txt");
		/* First write vertex onto file */
		boost::tie(vertex_It, vertex_End) = boost::vertices(gr);
		for(;vertex_It!=vertex_End;++vertex_It)
		{
			myfile<<"VERTEX2 ";
			myfile<<gr[*vertex_It].key;
			myfile<<" ";
			for (int i =0;i< 3;i++){
				myfile<<gr[*vertex_It].data[i];
				//std::cout<<pg.gr_[*pg.vertexIt_].data[i]<<std::endl;;
				myfile<<" ";
			}
			myfile<<"\n";
		
			std::cout<<std::endl;
		}

		/* Write edges to same file */	
		boost::tie(edge_It,edge_End) = boost::edges(gr);
		for(;edge_It!=edge_End;++edge_It)
		{
			double result;	
			//myfile<<*pg.vertexIt_<;
			myfile<<"EDGE2 ";
			myfile<<gr[*edge_It].src<<" "<<gr[*edge_It].obs<<" ";
			Eigen::Matrix4f trfMat = gr[*edge_It].transformation;
			result = acos(*(trfMat.data()));
			//According to the given file format:  forward, sideward, rotate, inf_ff, inf_fs, inf_ss, inf_rr,inf_fr,inf_sr
			myfile<<*(trfMat.data()+12)<<" "<<*(trfMat.data()+13)<<" "<<result<<" "<<"1 0 1 1 0 0";
			/*
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
		myfile.close();
		
		/*write edge scores to seperate file */
		/*
		file.open("edgeScores.txt");
		boost::tie(edge_It,edge_End) = boost::edges(gr);
		for(;edge_It!=edge_End;++edge_It)
		{
			//myfile<<*pg.vertexIt_<;
			file<<"Edge ";
			file<<gr[*edge_It].src<<" "<<gr[*edge_It].obs<<" "<<gr[*edge_It].score<<" ";
			Eigen::Matrix4f trfMat = gr[*edge_It].transformation;
			for(int i = 0;i<16;i++){
				file<< *(trfMat.data()+i)<< " ";
			} 
			/* 
			for(size_t i =0;i<=trfMat.rows();i++ ){
				for (size_t j = 0; j<=trfMat.cols();j++){
					myfile<< *(trfMat.data()+i+j)<< " ";;	
				}
			
			}
			//myfile<<pg.gr_[*pg.edgeIt_].transformation;
			file<< "\n";
		
		}*/
		file.close();
	
		//boost::write_graphviz(cout, gr);
	}


int main(int argc, char **argv)
{
	/*
	 * Initialize Ros components. Subscribe to point cloud , odometry.
	 */ 
	cout<<"Initializing ros node"<<endl;
	ros::init(argc,argv,"graph_node");

	Node graphNode;	
	/* Write data to file */

	
	return 0;
}

