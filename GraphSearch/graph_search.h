/**
 * @file graph_search.h
 * @brief backend of graph search, implementation of A* and JPS
 */

#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

#include <boost/heap/d_ary_heap.hpp> 
#include <memory>					 
#include <limits>					 
#include <vector>					 
#include <unordered_map>			 
#include <set>
#include <map>
#include <algorithm>
#include <cstdlib>
#include <fstream>


const short L=0, B=0, D=0;
const short R=1, F=2, U=4;

// indices of cluster parts sorted by position
// i.e. right-back-up point_3d RBU = 5
enum e_cluster_part
{
	LBD,	RBD,
	LFD,	RFD,
	LBU,	RBU,
	LFU,	RFU
};

enum e_state_desc
{
	UNSET,	 // default node state before initialization
	UNKNOWN, // excessive nodes (out of map range)
	WHITE,	 // free space
	BLACK,	 // obstacle
	GRAY	 // node containing both free space and obstacle, to be
};

struct point_3d
{
	int x, y, z = 0;

	point_3d() {}
	point_3d(int x_, int y_, int z_)
		: x(x_), y(y_), z(z_)
	{	}
	point_3d(const point_3d & _copy)
		: x(_copy.x), y(_copy.y), z(_copy.z)
	{	}

	friend std::ostream& operator << (std::ostream& os, const point_3d & val) {
		os << val.x << '-'<< val.y << '-' << val.z;
		return os;
	}
};

struct tree_node;
struct search_node;
using tree_node_ptr = std::shared_ptr<tree_node>;
using search_node_ptr = std::shared_ptr<search_node>;

/// Heap element comparator struct (as required in boost::heap::compare)
template <class T>
struct compare_states
{
	bool operator()(T a1, T a2) const
	{
		const double f1 = a1->g + a1->h;
		const double f2 = a2->g + a2->h;
		const double eps = 0.000001;
		
		// if equal compare gvals
		if ((f1 >= f2 - eps) && (f1 <= f2 + eps))
			 return a1->g < a2->g; 
		else return f1 > f2;
	}
};

using Astar_priority_queue = boost::heap::d_ary_heap<
	search_node_ptr,
	boost::heap::mutable_<true>,
	boost::heap::arity<2>,
	boost::heap::compare< compare_states<search_node_ptr> >
>;

/// Node of the octree
class tree_node
{
public:
	int id;
	point_3d coord;
	e_state_desc status = e_state_desc::UNSET;

	// "length" of cube edge
	short int size;
	// "volume" of cube, pow3(size)
	inline const int capacity() const;
	// amount of occupied map cells
	// store as int to keep statistical value when obstacle data is cleared
	int occupancy = 0;
	short int depth = 0;

	tree_node_ptr parent = nullptr;
	std::array<tree_node_ptr, 8> children;
	std::set<tree_node_ptr> neighbors;

	// info about obstacles from parent node (or whole map for root)
	// stored in array[8] to be further dispersed to children
	// as obstacles info is not required after tree construction (see status),
	// info is properly proceeded to children and current array is cleared after children generation
	std::array<std::vector<point_3d>, 8> obstacles;

	void connect_children();
	const bool is_neighbor_to(const tree_node_ptr& node) const;

	tree_node()
	: id(0), coord(0,0,0), size(0)
	{}

	tree_node(
		int _id, 
		point_3d _coord, 
		int _size, 
		tree_node_ptr _parent = nullptr
	)
	: coord(_coord), id(_id), size(_size), parent(_parent)
	{
		for (int i = 0; i < 8; i++)
		{
			obstacles[i] = std::vector<point_3d>();
			children [i] = nullptr;
		}
	}

	virtual ~tree_node() {}
};

/// Astar state of tree node in path search graph
class search_node : public tree_node
{
public:
	/// weighed connections
	std::map<search_node_ptr, double> neighbors_weights;
	/// list of paths to other nodes
	std::map<search_node_ptr, std::vector<int>> paths;
	/// node connections to neighboring cluster
	std::vector<int> interedges_ids;

	// some vars for HPA clustering
	short int cluster_id = -1;
	std::set<int> leads_to_clusters;

	Astar_priority_queue::handle_type heap_key;
	bool opened = false, closed = false;
	double h = 0, g = std::numeric_limits<double>::infinity();

	search_node_ptr prev_in_path = nullptr;

	search_node(const tree_node& copy)
	: tree_node(copy)
	{	}
	
	search_node(int _id, point_3d _coord, int _size = 0, tree_node_ptr _par = nullptr)
	: tree_node(_id, _coord, _size, _par)
	{	}

	~search_node() 
	{  }
};

class octree_path_planner
{
public:
	octree_path_planner(
		const char *map_data, 
		int x_dim, int y_dim, int z_dim, 
		short int recurse_lvl = 1, 
		double eps = 1, 
		bool verbose = false
	);

	void construct_octree();
	tree_node_ptr make_root_node(int &maxdepth);
	tree_node_ptr make_child_node(tree_node_ptr par, e_cluster_part _label, int _id);
	tree_node_ptr make_corner_node(tree_node_ptr par, e_cluster_part _label, int _id, int corner_size);

	search_node_ptr convert_to_search_node(tree_node_ptr t);

	static inline search_node_ptr cast(const tree_node_ptr& ref) {
		// derived is not polymorfic and all object must be initialized as derived, so static cast
		return std::static_pointer_cast<search_node>(ref);
	}
	
	int expand_iteration = 0;

	/// this function conducts path planning on the map
	/// max_expand: maximum number of expansion allowed, optional, default is -1, means no limitation
	bool plan(
		int start_x, int start_y, int start_z, 
		int goal_x, int goal_y, int goal_z, 
		int max_expand = -1
	);

	/// Get the optimal path for given start-goal
	std::vector<search_node_ptr> get_path() const;
	/// Get the search nodes in opened set
	std::vector<search_node_ptr> get_open_set() const;
	/// Get the search nodes in closed set
	std::vector<search_node_ptr> get_close_set() const;
	/// Get the whole search graph
	std::vector<search_node_ptr> get_all_set() const;

private:
	void set_node_status(tree_node_ptr node);
	void external_search(tree_node_ptr node);
	void mark_treebranch(int c_id, search_node_ptr &c);

	/// HPA Functional
	void set_cluster_insides(std::vector<search_node_ptr> &cluster_inside, tree_node_ptr &current);
	void cluster_dijkstra(std::vector<search_node_ptr> &cluster);
	void insert_node(search_node_ptr &S);
	bool Astar_graph_search();
	void set_Astar_graph(int cluster_depthlvl);

	/// Main planning loop
	bool plan(search_node_ptr &currNode_ptr, int max_expand);
	/// Get successor function for A*
	void get_successors(const search_node_ptr &curr, std::map<int, double> &succ_costs);
	/// Recover the optimal path
	std::vector<search_node_ptr> recover_path(search_node_ptr node);

	/// Calculate heuristic
	inline const double get_heuristic_weight(point_3d v) const;

	// map size
	point_3d dim;
	// int xDim_, yDim_, zDim_;

	// =1 by def
	double m_eps = 1;

	bool m_verbose;
	short m_with_corners = 0;
	// destination point
	point_3d m_start; int m_start_id;
	point_3d m_goal; int m_goal_id;
	void set_targets();

	// raw map data from yaml
	const char *m_map_data;

	// octree data <-> search graph

	tree_node_ptr m_root;
	std::vector<search_node_ptr> m_tree_nodes;
	std::map<int, search_node_ptr> m_search_nodes;

	Astar_priority_queue m_search_front;
	std::map<int, std::vector<search_node_ptr>> m_search_clusters;
	std::vector<bool> m_seen_list;

	//resulting path
	std::vector<search_node_ptr> m_raw_path;

	// helper fstreams to write out results
	std::ofstream fout1, fout2;
};

#endif
