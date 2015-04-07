// header file for poseGraph.cpp

#ifndef __POSEGRAPH_H_INCLUDED__  // if this header is not included yet
#define __POSEGRAPH_H_INCLUDED__  // then include this header

#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_map/property_map.hpp>
#include <Eigen/Dense>
#include <boost/graph/graphviz.hpp>

class PoseGraph{
	public:
		struct pose_{
			int key;
			std::vector<double>  data;
		};
		
		// Custom edge properties as constraints
		struct constraints_{
			int src_;
			int obs_;
			Eigen::Matrix4f transformation;
		};
		
		//Graph type 
		typedef boost::adjacency_list<boost::listS,boost::vecS, boost::undirectedS,pose_, constraints_> Graph_;
		typedef boost::graph_traits<Graph_>::vertex_descriptor Vertex_;
		typedef boost::graph_traits<Graph_>::edge_descriptor Edge_;
		Graph_ gr_;
		Graph_::vertex_iterator vertexIt_,vertexEnd_;
		Graph_::edge_iterator edgeIt_,edgeEnd_;
		//boost::tie(vertexIt, vertexEnd) = boost::vertices(pg.gr_);
		
		// Add vertex to the graph
			void addVertex(int v,std::vector<double> odom_);
		// Add edge in betwwen the last 2 vertices of the graph
			void addEdgeToPrev(Eigen::Matrix4f tr_msg);
		// Print the graph
			void display();
};

#endif
