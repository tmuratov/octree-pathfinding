/**
 * @file graph_search.h
 * @brief backend of graph search, implementation of A* and JPS
 */

#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

#include <boost/heap/d_ary_heap.hpp> // boost::heap::d_ary_heap
#include <memory>					 // std::shared_ptr
#include <limits>					 // std::numeric_limits
#include <vector>					 // std::vector
#include <unordered_map>			 // std::unordered_map
#include <set>
#include <map>
#include <algorithm>
#include <cstdlib>
#include <fstream>

enum e_dir_code{
	L=0,
	R=1,
	B=0,
	F=2,
	D=0,
	U=4
};

// indices of cluster parts sorted by position
// i.e. right-back-up point_3d RBU = 5
enum e_cluster_part
{
	NONE = -1,
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
		: x(x_), y(y_), z(x_)
	{	}
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
		return f1 > f2;
	}
};

using Astar_priority_queue = boost::heap::d_ary_heap<
	search_node_ptr,
	boost::heap::mutable_<true>,
	boost::heap::arity<2>,
	boost::heap::compare< compare_states<search_node_ptr> >
>;

/// Node of the octree
struct tree_node
{
	point_3d coord;
	int id;
	e_state_desc status = e_state_desc::UNSET;
	e_cluster_part label = e_cluster_part::NONE;

	// "length" of cube edge
	int size;
	// "volume" of cube
	int capacity;
	// amount of occupied map cells
	int occupancy = 0;

	short int depth;
	int cluster_id = -1;

	search_node_ptr Astar_ref = nullptr;

	tree_node_ptr parent = nullptr;
	std::vector<tree_node_ptr> children;
	std::vector<tree_node_ptr> neighbors;

	// info about obstacles from parent node (or whole map for root)
	// stored in array[8] to be further dispersed to children
	// as obstacles info is not required after tree construction (see status),
	// info is properly proceeded to children and current array is cleared after children generation
	std::array<std::vector<point_3d>, 8> obstacles;

	void connect_children();
	int get_status() const;
	bool access_neighboring(tree_node_ptr node);

	tree_node(int _id, int _size, int _x, int _y, int _z, tree_node_ptr _parent = nullptr, e_cluster_part _label = e_cluster_part::NONE)
	: coord(_x, _y, _z), id(_id), size(_size), parent(_parent), label(_label)
	{
		children.reserve(8);
	}
};

/// Astar state of tree node in path search graph
struct search_node
{
	point_3d coord;
	int id;
	int dx, dy, dz = 0;

	/// reference to an octree node
	tree_node_ptr octree_ref = nullptr;
	/// weighed connections
	std::map<search_node_ptr, double> neighbors;
	std::set<int> neighbor_ids;
	/// list of paths to other nodes
	std::map<search_node_ptr, std::vector<int>> paths;
	/// node connections to neighboring cluster
	std::vector<int> interedges_ids;

	std::set<int> leads_to_clusters;
	int cluster_id = -1;

	Astar_priority_queue::handle_type heap_key;

	bool opened = false;
	bool closed = false;
	double g = std::numeric_limits<double>::infinity();
	double h = 0;

	search_node_ptr prev_in_path = nullptr;

	search_node(int _id, int x, int y, int z, int _dx, int _dy, int _dz )
	: coord(x, y, z), id(_id), dx(_dx), dy(_dy), dz(_dz)
	{}
};

class octree_path_planner
{
public:
	octree_path_planner(const char *map_data, int x_dim, int y_dim, int z_dim, short int recurse_lvl = 1, double eps = 1, bool verbose = false);

	void construct_octree();
	tree_node_ptr make_root_node(int &maxdepth);
	tree_node_ptr make_child_node(tree_node_ptr p, int _label, int _id);
	tree_node_ptr make_corner_node(tree_node_ptr p, int _label, int _id, int corner_size);

	search_node_ptr convert_to_graph_node(tree_node_ptr t);

	int expand_iteration = 0;

	/// this function conducts path planning on the map
	/// max_expand: maximum number of expansion allowed, optional, default is -1, means no limitation
	bool plan(int start_x, int start_y, int start_z, int goal_x, int goal_y, int goal_z, int max_expand = -1);

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
	void mark_treebranch(int c_id, tree_node_ptr &c);

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

	/// Get subscript
	int coord_to_id(point_3d v) const;
	/// Check if (x, y, z) is free
	bool is_free(point_3d v) const;
	/// Check if (x, y, z) is occupied
	bool is_occupied(point_3d v) const;
	/// Calculate heuristic
	double get_heuristic_weight(point_3d v) const;

	// map size
	std::array<int, 3> dim;
	// int xDim_, yDim_, zDim_;

	// =1 by def
	double m_eps = 1;

	bool m_verbose;

	// destination point
	point_3d m_start; int m_start_id;
	point_3d m_goal; int m_goal_id;
	void set_targets();

	// raw map data from yaml
	const char *m_map_data;

	// octree data <-> search graph
	std::vector<tree_node_ptr> m_tree_nodes;
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
