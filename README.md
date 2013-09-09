adaptable-a-star
==============

Summary
---

Adaptable and efficient A* graph search and pathfinding implementation that operates on every collection that provides forward iteration (such as `std::deque`, `std::list`, custom collections, you name it).

Thanks to the templated design it's suited to work with every graph with positive weighted nodes or edges, little work required. 

Usage
---

Include `AStar.hpp` and create a specialization of `AStar::NodeBase`. 

For instance, if you choose to store your nodes in a `std::deque`, the specialization could look like the following:
````
class MyNode: public AStar<MyNode, std::deque>::NodeBase {
public:
  const double distance(const MyNode *rhs) const { /*...*/ }
  const double heuristic(const MyNode *rhs) const { /*...*/ }
  const Collection successors(const Iterator &collection_begin, const Iterator &collection_end) const { /*...*/ }
  const bool operator==(const MyNode *rhs) const { /*...*/ }
}
````

To perform a search, simply instantiate AStar with the iterators `collection.begin`, `collection.end`, `path_begin` and `path_end`. The AStar object then holds the optimal path, ready to be traversed.
````
std::deque<MyNode*> nodes;
AStar<MyNode, std::deque> myAStar(nodes.begin(), nodes.end(), path_begin, path_end);
````

A reference implementation using a two-dimensional, rectangular, evenly distributed grid (or, with other words, a simple `Tile Collision Map`) with extensive documentation can be found in `Demo.cpp`.

Compilation
---
Compiling the source is easy. Use one of the following console commands...
````
clang++ -std=c++11 -stdlib=libc++ Demo.cpp -o astar_demo

g++ -std=c++11 Demo.cpp -o astar_demo
````
..., include `AStar.hpp` in your existing project, or utilize your favourite IDE. (No, DevC++ is not an IDE you should use.)


License
---
This software is licensed under the zlib/libpng license. It's free to use this software in non-commercial as well as in commercial projects. 

See `LICENSE` in the project root directory for details.
