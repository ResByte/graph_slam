/*
 * poseGraph.cpp
 * Description: This is the class for pose graph construction.
 * 				It is used for the development of pose graph SLAM in robotics.
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

#include "poseGraph.h"

/*
 * This Function adds the vertex to the graph with the parameter as index of the vertex
 */ 
	
void PoseGraph::addVertex(int v, std::vector<double> odom_){
	std::cout << "-->Adding vertex "<<std::endl;
	Vertex_ v1;
	v1 = boost::add_vertex(gr_);
	gr_[v1].key = v ;
	gr_[v1].data = odom_;
	return;
	}

/*
 * This function adds an edge between the last 2 add vertices.
 * The parameter taken is the transformation matrix. 
 */ 
void PoseGraph::addEdgeToPrev(Eigen::Matrix4f tr_msg){
	std::cout<<"-->Adding edge to the prev"<< std::endl;
	Graph_::vertex_iterator vertexIt,vertexEnd;
	boost::tie(vertexIt, vertexEnd) = boost::vertices(gr_);
	std::cout<< "-->Vertex End is :"<<*vertexEnd<<std::endl;
	Edge_ e1; 
	e1 = (boost::add_edge(*(vertexEnd-2),(*vertexEnd-1),gr_)).first;
	gr_[e1].transformation = tr_msg;
	gr_[e1].src_ = gr_[*vertexEnd-2].key;
	gr_[e1].src_ = gr_[*vertexEnd-1].key;
	std::cout<<"previous vertices "<< gr_[*vertexEnd-2].key <<std::endl;  
	
	return;
	}
	
	
/*
 * This function print out the current graph.
 */ 
void PoseGraph::display(){
	boost::write_graphviz(std::cout, gr_);
	return;
	}


