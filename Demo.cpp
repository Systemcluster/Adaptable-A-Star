//
// A* graph search and pathfinding implementation test/demonstration
//
// Created by Christian Sdunek, 2013-09-07
// Copyright (c) 2013 Christian Sdunek.
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//

#include "AStar.hpp"
#include <deque>
#include <cmath>

#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>


// the size of the rectangular graph
const unsigned int world_width = 5;
const unsigned int world_height = 10;

/**
 * A basic AStar::NodeBase implementation using evenly distributed (x, y) coordinates as positions.
 * Simple implementations like this could for example be utilized for tile-collision based 2D games.
 *
 * Note: This node implementation holds no pointers to its neighbours as they are easily computable
 * based on the nodes position and the size of the graph. See @successors and @world1 below for reference.
 *
 * Internal types based on the AStar<node_type, collection_type> template arguments:
 *   Collection (collection_type)
 *   Iterator   (collection_type::iterator)
 */
class MyNode: public AStar<MyNode, std::deque>::NodeBase {
private:
	/**
	 * The (x, y) position in the graph.
	 */
	double x,y;
public:
	/**
	 * Constructor taking the nodes position and if it is occupied.
	 */
	MyNode(double x, double y, bool blocked = false):
	x(x), y(y) {
		available = !blocked;
	}
	~MyNode() {}
	
	/**
	 * Returns ths real distance between this node and @rhs.
	 * @rhs is assured to be a neighbour of this node.
	 *
	 * As in this demo implementation the neighbour nodes are all exactly one unit apart,
	 * a constant value of one (1) could be returned here. Therefore the calculation is currently merely implemented
	 * for demonstration purposes, but it would be required for diagonal node connections/edges.
	 */
	virtual const double distance(const MyNode *rhs) const {
		double x_dist = x - rhs->x;
		double y_dist = y - rhs->y;
		return std::sqrt(x_dist * x_dist + y_dist * y_dist);
	}
	/**
	 * Returns ths estimated, heuristic distance between this node and @rhs.
	 * Must not be greater than the actual path. The linear distance is a common values for this purpose.
	 */
	virtual const double heuristic(const MyNode *rhs) const {
		// using the linear distance as heuristic
		return distance(rhs);
	}
	
	/**
	 * Returns this nodes successors (or, the nodes this node has active edges to).
	 * The successors in this specific graph are simply the neighbours one unit apart in each direction.
	 */
	virtual const Collection successors(const Iterator &collection_begin, const Iterator &collection_end) const {
		Collection ret;
		
		// Calculate the neighbour positions
		double en {(x + 1 + world_width * (y    ))}; // Eastern neighbour
		double wn {(x - 1 + world_width * (y    ))}; // Western neighbour
		double sn {(x     + world_width * (y + 1))}; // Southern neighbour
		double nn {(x     + world_width * (y - 1))}; // Northern neighbour
		
		// if the calculated neighbours are in the bounds of the graph,
		// add them to the returned collection.
		if(0 <= en && en < world_height * world_width && x != world_width-1) {
			auto e = collection_begin [en];
			ret.push_back(e);
		}
		if(0 <= wn && wn < world_height * world_width && x != 0) {
			auto e = collection_begin [wn];
			ret.push_back(e);
		}
		if(0 <= sn && sn < world_height * world_width && y != world_height-1) {
			auto e = collection_begin [sn];
			ret.push_back(e);
		}
		if(0 <= nn & nn < world_height * world_width && y != 0) {
			auto e = collection_begin [nn];
			ret.push_back(e);
		}
		
		return ret;
	}
	
	/**
	 * Returns if this node equals @rhs. In the graph used in this demonstration,
	 * a node is simply characterized through its position.
	 *
	 * The slightly 'inexact' comparison is required thanks to standard floating-point inaccuracy.
	 */
	virtual const bool operator==(const MyNode *rhs) const {
		return std::abs(x - rhs->x) < std::numeric_limits<double>::epsilon() &&
		std::abs(y - rhs->y) < std::numeric_limits<double>::epsilon();
	}
	
	/**
	 * Convenience function returning a nicely formatted string of the node properties.
	 */
	virtual const std::string str() const {
		std::stringstream ss;
		ss << std::fixed;
		ss << std::setprecision(2);
		ss << x;
		ss << " ";
		ss << y;
		ss << " g(";
		ss << g;
		ss << ") f(";
		ss << f;
		ss << ")";
		return ss.str();
	}
};

/**
 * The test graph.
 *
 * Holds (@world_width * @world_height) elements.
 * Each node is initialized with (x, y, blocked) attributes.
 *
 * To compare, the shortest path ('?' are ambiguous) is:
 *
 * o   #
 *     #
 *     #   #
 *         #   ?
 *         ?   #
 *             #
 *         ?   #
 *         #   ?
 *         #
 *         #   #   x
 *
 */
std::deque<MyNode> world1 {{
	{ 0, 0, 0 },{ 1, 0, 0 },{ 2, 0, 1 },{ 3, 0, 0 },{ 4, 0, 1 },
	{ 0, 1, 1 },{ 1, 1, 0 },{ 2, 1, 1 },{ 3, 1, 0 },{ 4, 1, 0 },
	{ 0, 2, 1 },{ 1, 2, 0 },{ 2, 2, 0 },{ 3, 2, 1 },{ 4, 2, 1 },
	{ 0, 3, 1 },{ 1, 3, 1 },{ 2, 3, 0 },{ 3, 3, 0 },{ 4, 3, 1 },
	{ 0, 4, 0 },{ 1, 4, 0 },{ 2, 4, 0 },{ 3, 4, 0 },{ 4, 4, 0 },
	{ 0, 5, 0 },{ 1, 5, 1 },{ 2, 5, 1 },{ 3, 5, 0 },{ 4, 5, 0 },
	{ 0, 6, 0 },{ 1, 6, 0 },{ 2, 6, 0 },{ 3, 6, 0 },{ 4, 6, 0 },
	{ 0, 7, 0 },{ 1, 7, 0 },{ 2, 7, 0 },{ 3, 7, 0 },{ 4, 7, 1 },
	{ 0, 8, 0 },{ 1, 8, 1 },{ 2, 8, 0 },{ 3, 8, 1 },{ 4, 8, 0 },
	{ 0, 9, 0 },{ 1, 9, 1 },{ 2, 9, 0 },{ 3, 9, 0 },{ 4, 9, 0 }
}};

/**
 * Small AStar demonstration and test run.
 *
 *
 */
int main(int argc, const char * argv[]) {
	// Create a collection of pointers to the MyNode elements
	std::deque<MyNode*> world{};
	for(auto &elem: world1) {
		world.push_back(&elem);
	}
	
	// Perform the A* search on @world from the upper left to the lower right node
	AStar<MyNode, std::deque> myAStar(world.begin(), world.end(),
									  world.begin(), (world.begin()+(world.size()-1)));
	
	// Output the shortest path if one exists
	if(myAStar.successful()) {
		for(auto elem: myAStar) {
			std::cout << elem.str() << std::endl;
		}
		std::cout << "Shortest path found with " << myAStar.weight() << " weight." << std::endl;
	}
	else std::cout << "No existing path." << std::endl;
	
	return 0;
}
