/*
 * local_node.cpp
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
 * This is written according to ROS c++ guidelines
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
#include <pcl/filters/voxel_grid.h>
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
#include <octomap/OcTreeBase.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
typedef pcl::PointCloud<pcl::PointXYZ> pcl_cld;
typedef pcl_cld::Ptr pcl_cld_ptr; 
typedef octomap::OcTree::leaf_iterator it_t; // global leaf iterator
const float res = 0.1;
using namespace std;
/* callback function for cloud msg */

class Node{
    public:
        float threshold_distance; 
        ros::NodeHandle nh;
		ros::Subscriber odom;
		ros::Subscriber cloud;
		ros::Publisher octo_pub; // publisher for octomap
		
		double pose_x,pose_y,roll,pitch,yaw;
		pcl::PCLPointCloud2::Ptr cloud_ptr;
		pcl::PCLPointCloud2 cloud_ ; 
		pcl::PointCloud<pcl::PointXYZ> curr_pc;
		Eigen::Matrix4f tr_mat;
		Eigen::Matrix4f prev_tr_mat;
		
		std::deque< pcl::PointCloud<pcl::PointXYZ> > cloud_seq_loaded;
		octomap::OcTree* tree;

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
		struct odometry_{
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
                /* Random sampling of points */
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

                /* Generic subscribing script */
		void getData();
                /* For Octomap publishing and display on Rviz  */
		std_msgs::ColorRGBA getColorByHeight(double h);
		void publishOctomap(octomap::OcTree* map);
                /* Evaluate validity of transform */
		double evaluateValidTransform(pcl_cld_ptr source,pcl_cld_ptr target);
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
	
    gicp.setInputSource(first_pc);
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

void Node::getData(){
	this->odomSub();
	this->cloudSub();
	ros::spinOnce();

}		

// This function comes from the octomap_server pkg
std_msgs::ColorRGBA Node::getColorByHeight(double h) {
  double range = 5.05;
  h = 1.0 - std::min(std::max(h/range, 0.0), 1.0);
  h *= 0.8;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}

void Node::publishOctomap(octomap::OcTree* tree_map){
	visualization_msgs::MarkerArray msg;
	msg.markers.resize(tree_map->getTreeDepth()+1);

	it_t it = tree_map->begin_leafs();
	it_t end = tree_map->end_leafs();
	// For leaf in leaves
	for (; it != end; ++it) {
		// If occupied
		if (tree_map->isNodeOccupied(*it)) {
			// Get some info about the leaf
			double x = it.getX();
			double y = it.getY();
			double z = it.getZ();
			//std::cout<<x<<" "<<y<<" "<<z<<std::endl;
			
			size_t depth = it.getDepth();
			// Insert a point for the leaf's cube
			geometry_msgs::Point leaf_origin;
			leaf_origin.x = x;
			leaf_origin.y = y;
			leaf_origin.z = z;
			msg.markers[depth].points.push_back(leaf_origin);
			// Determine and set the leaf's color by height
			//std::cout<<"determining color by height"<<std::endl;
			msg.markers[depth].colors.push_back(getColorByHeight(leaf_origin.z));
			
		}
	}
	std::cout<<" Finish the marker array setup"<<std::endl;
	std_msgs::ColorRGBA color;
	color.a = 1.0;
	for (size_t i = 0; i < msg.markers.size(); ++i) {
		double size = tree_map->getNodeSize(i);
		msg.markers[i].header.frame_id = "/camera_depth_frame";
		msg.markers[i].header.stamp = ros::Time::now();
		msg.markers[i].ns = "map";
		msg.markers[i].id = i;
		msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		msg.markers[i].scale.x = size;
		msg.markers[i].scale.y = size;
		msg.markers[i].scale.z = size;
		msg.markers[i].action = visualization_msgs::Marker::ADD;
		msg.markers[i].color = color;
	}
	 std::cout<<"Publish the marker array"<<std::endl;
	
    this->octo_pub.publish(msg) ;
}

double Node::evaluateValidTransform(pcl_cld_ptr source,pcl_cld_ptr target){

	/* evaluate the validity of edge transforms */
			
	pcl::registration::TransformationValidationEuclidean<pcl::PointXYZ, pcl::PointXYZ> tve;
	tve.setMaxRange (0.1); // 1cm
	double score = tve.validateTransformation (source, target, tr_mat);
	return score;		
}
		
/* main node process as well as constructor*/ 
Node::Node()
{	
	this->tree = new octomap::OcTree(res);
	pcl::PointCloud<pcl::PointXYZ> prev_pc; // one step buffer for clouds comparision
	int count = 1; // counter for key of the vertex
	threshold_distance=0.15; // thesholding for the distance travelled
	std::vector<double> curr_odom; // vector for current value of odometry
	this->octo_pub = nh.advertise<visualization_msgs::MarkerArray>("/map_vis", 1);
	// Initialize the first values 
	if (count ==1)
	{
		/* code */
		this->getData();
		count++;
	}


	/* Initialize graph with first vertex as the starting point */
	curr_odom.push_back(pose_x);
	curr_odom.push_back(pose_y);
	curr_odom.push_back(yaw);
	
	//cout<<"Initialize graph with first vertex as the starting point"<<endl;
	this->initGraph(0,curr_odom); 

	prev_pc = curr_pc;
	
	while(ros::ok())
	{
		this->getData();	
		double distance;
		curr_odom[0] = pose_x;
		curr_odom[1] = pose_y;
		curr_odom[2] = yaw;
		//cout<<"calculating prev odom value"<<endl;
		boost::tie(vertex_It,vertex_End) = boost::vertices(gr);
		
		std::vector<double> prev_odom;
		prev_odom = gr[*vertex_End-1].data;
		//cout<<"Calculate distance"<<endl;
		distance = this->calculateDist(curr_odom, prev_odom);
		//cout<<distance<<endl;
		
		/* if distance is greater than threshold */
		if(distance >= threshold_distance && curr_pc.size() > 0)
		{
			/*  add vertex to graph  */
			//cout<<"-->add vertex to graph"<<endl;
			
			this->addVertex(count,curr_odom);
			/* calculate transform */
			octomap::Pointcloud oct_pc;
			tr_mat =this->estTrans(curr_pc,prev_pc);
			for(int i = 0;i<curr_pc.points.size();i++){
				oct_pc.push_back((float) curr_pc.points[i].x,(float) curr_pc.points[i].y,(float) curr_pc.points[i].z);
			}
			octomap::point3d origin(float(pose_x),float(pose_y),0.0f);
			octomap::pose6d pose(float(pose_x),float(pose_y),0.0f,float(roll),float(pitch),float(yaw));
			tree->insertPointCloud(oct_pc,origin,pose,-1,false,false);
			std::cout<<"publishing octomap as MarkerArray"<<std::endl;
			this->publishOctomap(tree);
			boost::tie(vertex_i, vertex_e) = boost::vertices(gr);
			this->addEdge(tr_mat,0.000);
			this->detectLoopClosure(*(vertex_e-1));
			prev_pc = curr_pc;
			count++;
		}
		
		ros::spinOnce();
	}
		
	}



int main(int argc,char **argv){
	//Initialize ros node
	ros::init(argc,argv,"graph_node");
	Node graphNode;	
	return 0;	
	}
