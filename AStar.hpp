//
// A* graph search and pathfinding implementation
//
// Created by Christian Sdunek on 07.09.13.
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

#ifndef SCU_ASTAR_001_HPP
#define SCU_ASTAR_001_HPP

#include <iterator>
#include <set>

#include <limits>

/**
 * A* (A-Star) graph search algorithm implementation.
 *
 * Template parameters:
 *   node_type:       Type representing a node. Must implement AStar::NodeBase.
 *   collection_type: Any collection that provides at least std::forward_iterator_tag iteration
 */

template <typename node_type,
          template <typename...> class collection_type>
class AStar {
public:
	class NodeBase;
	class ResultIterator;
	
	typedef node_type                     Node;
	typedef collection_type<Node*>        Collection;
	typedef typename Collection::iterator Iterator;
	
private:
	
	struct NodePointerEqual {
		NodePointerEqual(Node *lhs): lhs(lhs) {}
		const Node *lhs;
		const bool operator()(const Node *rhs) const {
			return lhs->operator==(rhs);
		}};
	struct NodePointerLess {
		const bool operator()(const Node *lhs, const Node *rhs) const {
			return lhs->operator< (rhs);
		}};
	struct NodePointerLessHeuristicDistance {
		const bool operator()(const Node *lhs, const Node *rhs) const {
			return lhs->h < rhs->h;
		}};
	
	typedef std::multiset<Node*, NodePointerLess> OpenList;
	typedef std::multiset<Node*, NodePointerLessHeuristicDistance> ClosedList;
	typedef typename OpenList::iterator ListIterator;
	
	OpenList openList {};
	ClosedList closedList {};
	
	Iterator collection_begin;
	Iterator collection_end;
	Iterator path_begin;
	Iterator path_end;
	
	void calculate();
	void expand   (Node *current, Node* successor);
	void backlink (Node *first, Node *last);
	
public:
	AStar(Iterator collection_begin,
		  Iterator collection_end,
		  Iterator path_begin,
		  Iterator path_end);
	virtual ~AStar();
	
	const bool         successful() const;
	const double       weight() const;
	const unsigned int steps() const;
	
	const ResultIterator begin() const;
	const ResultIterator end() const;
};

template<typename node_type, template <typename...> class collection_type>
class AStar<node_type, collection_type>::NodeBase {
	friend AStar;
	friend NodePointerLessHeuristicDistance;
protected:
	double       g {0};
	double       f {0};
	double       h {-1};
	bool         available {1};
	unsigned int step {0};
	
	Node *prev {nullptr};
	Node *next {nullptr};
	
	typedef Collection Collection;
	typedef Iterator Iterator;
	
public:
	virtual ~NodeBase() {};
	
	virtual const double     distance(const Node *rhs) const = 0;
	virtual const double     heuristic(const Node *rhs) const = 0;
	virtual const Collection successors(const Iterator &collection_begin, const Iterator &collection_end) const = 0;
	
	virtual const bool operator< (const Node *rhs) const final {
		return f < rhs->f;
	};
	virtual const bool operator==(const Node *rhs) const = 0;
};

template<typename node_type, template <typename...> class collection_type>
class AStar<node_type, collection_type>::ResultIterator: public std::iterator<std::forward_iterator_tag, Node> {
	friend AStar;
private:
	Node *next {nullptr};
	ResultIterator(Node* next):next(next) {}
public:
	ResultIterator() {}
	ResultIterator(const ResultIterator &rhs): next(rhs.next) {}
	ResultIterator &operator=(const ResultIterator &rhs) {
		next = rhs.next;
	}
	const bool operator==(const ResultIterator &rhs) const {
		return (!next && !rhs.next) || (next && rhs.next && next->operator==(rhs.next));
	}
	const bool operator!=(const ResultIterator &rhs) const {
		return !operator==(rhs);
	}
	Node &operator*() {
		return *next;
	}
	Node *operator->() {
		return next;
	}
	ResultIterator &operator++() {
		next = next->next;
		return *this;
	}
};

template<typename node_type, template <typename...> class collection_type>
AStar<node_type, collection_type>::AStar(Iterator collection_begin,
										 Iterator collection_end,
										 Iterator path_begin,
										 Iterator path_end):
collection_begin(collection_begin), collection_end(collection_end),
path_begin(path_begin), path_end(path_end) {
	calculate();
	backlink(*++closedList.begin(), *path_begin);
}

template<typename node_type, template <typename...> class collection_type>
AStar<node_type, collection_type>::~AStar() {
	
}

template<typename node_type, template <typename...> class collection_type>
void AStar<node_type, collection_type>::calculate() {
	openList.insert(*path_begin);
	while(!openList.empty()) {
		auto currentIterator = openList.begin();
		Node *current = *currentIterator;
		
		openList.erase(currentIterator);
		closedList.insert(current);
		
		if(current == *path_end) {
			// FOUND
			break;
		}
		
		Collection successors = current->successors(collection_begin, collection_end);
		if(!successors.empty()) for(auto successor = successors.begin(); successor != successors.end(); ++successor) {
			expand(current, *successor);
		}
	}
	// NOT FOUND
}

template<typename node_type, template <typename...> class collection_type>
void AStar<node_type, collection_type>::expand(Node *current, Node *successor) {
	if(std::find_if(closedList.begin(), closedList.end(), NodePointerEqual(successor)) != closedList.end()) {
		return;
	}
	if(!successor->available) {
		return;
	}
	double g = current->g + current->distance(successor);
	ListIterator position = std::find_if(openList.begin(), openList.end(), NodePointerEqual(successor));
	if(position != openList.end()) {
		if(g > successor->g || (-std::numeric_limits<double>::epsilon() < (g - successor->g) &&
								(g - successor->g) < std::numeric_limits<double>::epsilon())) {
			return;
		}
		position = openList.erase(position);
	}
	else {
		successor->h = successor->heuristic(*path_end);
	}
	successor->prev = current;
	successor->g = g;
	successor->f = successor->h + g;
	successor->step = current->step + 1;
	
	openList.insert(position, successor);
}

template<typename node_type, template <typename...> class collection_type>
void AStar<node_type, collection_type>::backlink(Node *first, Node *last) {
	for (;first->prev; first = first->prev) {
		first->prev->next = first;
	}
}

template<typename node_type, template <typename...> class collection_type>
const bool AStar<node_type, collection_type>::successful() const {
	return *++closedList.begin() == *path_end;
}
template<typename node_type, template <typename...> class collection_type>
const double AStar<node_type, collection_type>::weight() const {
	return (*++closedList.begin())->g;
}
template<typename node_type, template <typename...> class collection_type>
const unsigned int AStar<node_type, collection_type>::steps() const {
	return (*++closedList.begin())->step;
}

template<typename node_type, template <typename...> class collection_type>
const typename AStar<node_type, collection_type>::ResultIterator AStar<node_type, collection_type>::begin() const {
	return ResultIterator(*path_begin);
}
template<typename node_type, template <typename...> class collection_type>
const typename AStar<node_type, collection_type>::ResultIterator AStar<node_type, collection_type>::end() const {
	return ResultIterator();
}


#endif
