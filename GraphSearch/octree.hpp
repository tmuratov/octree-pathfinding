#include "graph_search.hpp"

void connect(tree_node_ptr A, tree_node_ptr B){
	if (!A || !B) return;
	A->neighbors.insert(B);
	B->neighbors.insert(A);
}
void tree_node::connect_children()
{
	if (children.empty())
	{
		std::cout << "no children to connect!" << std::endl;
		return;
	}
	else
	{
		// templated neighbors
		for (int i = 0; i < 8; i++)
		{
			if (!children[i]) continue;
			children[i]->neighbors.clear();
			connect(children[i], children[i]->parent);
		}
			// no diagonal connections, make 6-direction grid
			connect(children[LBD], children[RBD]);
			connect(children[LBD], children[LFD]);
			connect(children[LBD], children[LBU]);

			connect(children[RBD], children[LBD]);
			connect(children[RBD], children[RFD]);
			connect(children[RBD], children[RBU]);
			
			connect(children[LFD], children[RFD]);
			connect(children[LFD], children[LBD]);
			connect(children[LFD], children[LFU]);
			
			connect(children[RFD], children[LFD]);
			connect(children[RFD], children[RBD]);
			connect(children[RFD], children[RFU]);
			
			connect(children[LBU], children[RBU]);
			connect(children[LBU], children[LFU]);
			connect(children[LBU], children[LBD]);
			
			connect(children[RBU], children[RFU]);
			connect(children[RBU], children[LBU]);
			connect(children[RBU], children[LFD]);
			
			connect(children[LFU], children[RFU]);
			connect(children[LFU], children[LBU]);
			connect(children[LFU], children[LFD]);
			
			connect(children[RFU], children[LFU]);
			connect(children[RFU], children[RBU]);
			connect(children[RFU], children[RFD]);
	}
}
void octree_path_planner::external_search(tree_node_ptr node)
{
	if (node == nullptr) return;

	for (auto it = node->parent->neighbors.begin(); it != node->parent->neighbors.end(); it++)
	{
		if ((*it) == nullptr) continue;
		bool are_neighbors = node->is_neighbor_to(*it);
		if (are_neighbors)
		{
			connect(*it, node);
		}
	}
}

bool tree_node::is_neighbor_to(tree_node_ptr n) const
{
	int ox = -5, oy = -5, oz = -5; // overlap result for each axis

	if (this->coord.x + this->size == n->coord.x ||
		n->coord.x + n->size == this->coord.x)
		ox = 1;
	else if (this->coord.x + this->size > n->coord.x &&
			 n->coord.x + n->size > this->coord.x)
		ox = 2;

	if (this->coord.y + this->size == n->coord.y ||
		n->coord.y + n->size == this->coord.y)
		oy = 1;
	else if (this->coord.y + this->size > n->coord.y &&
			 n->coord.y + n->size > this->coord.y)
		oy = 2;

	if (this->coord.z + this->size == n->coord.z ||
		n->coord.z + n->size == this->coord.z)
		oz = 1;
	else if (this->coord.z + this->size > n->coord.z &&
			 n->coord.z + n->size > this->coord.z)
		oz = 2;

	return (ox + oy + oz == 5);
}

void octree_path_planner::set_node_status(tree_node_ptr node)
{
	if (node->occupancy == 0)
	{
		if (node->coord.x >= dim.x && 
			node->coord.y >= dim.y && 
			node->coord.z >= dim.z
		)
			node->status = e_state_desc::UNKNOWN; // node out of map range --- unknown
		else
			node->status = e_state_desc::WHITE; // node is free to move through
	}
	else if (node->occupancy == node->capacity())
		node->status = e_state_desc::BLACK; // node is completely blocked, no need to divide further
	else
		node->status = e_state_desc::GRAY; // node contains obstacles, need to be further divided,
}

tree_node_ptr octree_path_planner::make_root_node(int &maxdepth)
{
	// define tree size: get closest ceiled pow2 to max dimension of raw map
	int max_dim = 0; 
	for (const int & dim_i : {dim.x, dim.y, dim.z})
		if (dim_i > max_dim) 
			max_dim = dim_i;
 	
	int treeside = 2; 	
	maxdepth = 1;
	while (treeside < max_dim)	{
		treeside *= 2;
		maxdepth++;
	}

	// ID: 0; node pos: (0,0,0) === LeftBackDown corner
	tree_node_ptr root = std::make_shared<tree_node>(0, treeside, point_3d(0,0,0), nullptr);

	for (int x = 0; x < dim.x; x++)
	for (int y = 0; y < dim.y; y++)
	for (int z = 0; z < dim.z; z++)
	{
		int ind = x + y*dim.x + z*dim.x*dim.y;
		if (m_map_data[ind] > 0)
		{
			short xl = L, yl = B, zl = D;
			// side is pow2, so int
			xl = x >= treeside / 2 ? R : L;
			yl = y >= treeside / 2 ? F : B;
			zl = z >= treeside / 2 ? U : D;
			root->occupancy++;
			root->obstacles[encode_label(xl,yl,zl)].push_back(point_3d(x, y, z));
		}
	}

	for (auto &sector : root->obstacles)
		sector.shrink_to_fit();

	delete [] m_map_data;
	m_map_data = nullptr;

	set_node_status(root);

	if (m_verbose)
	{
		std::cout << "\n------ root node ------" << std::endl;
		std::cout << "Max dimension  : " << max_dim  << std::endl;
		std::cout << "Root node size : " << treeside << std::endl;
		std::cout << "Tree excess n  : " << root->capacity() - dim.x*dim.y*dim.z << std::endl;
		std::cout << "Map occupancy  : " << root->occupancy << std::endl;
		for (int i = 0; i < root->obstacles.size(); i++)
			std::cout << "Sector " << i <<" size  : " << root->obstacles[i].size() << std::endl;
		std::cout << "Max depth level: " << maxdepth << std::endl;
		std::cout << "------ root node ------\n" << std::endl;
	}
	return root;
}

tree_node_ptr octree_path_planner::make_child_node(tree_node_ptr par, e_cluster_part _label, int _id)
{
	// size is pow2, so int
	int _size = par->size / 2;

	point_3d par_coord(par->coord);
	point_3d tmp_label = decode_label(_label);

	if (tmp_label.x == R) par_coord.x += _size;
	if (tmp_label.y == F) par_coord.y += _size;
	if (tmp_label.z == U) par_coord.z += _size;

	tree_node_ptr child = std::make_shared<tree_node>(_id, _size, par_coord, par);

	child->occupancy = par->obstacles[_label].size();
	child->depth = par->depth + 1;
	child->status = e_state_desc::UNSET;


	if (child->occupancy > 0)
	{
		// divide this child cube in 8 to filter obstacles
		// side is pow2, so int
		int Cx = par_coord.x + _size/2;
		int Cy = par_coord.y + _size/2;
		int Cz = par_coord.z + _size/2;

		// filter parent obstacles to child's sectors
		for (auto it = par->obstacles[_label].begin(); it != par->obstacles[_label].end(); /*++ on erase op*/)
		{
			point_3d obs(*it);
			short xl=L, yl=B, zl=D;

			xl = obs.x >= Cx ? R : L;
			yl = obs.y >= Cy ? F : B;
			zl = obs.z >= Cz ? U : D;
			//child->occupancy++;
			child->obstacles[encode_label(xl,yl,zl)].push_back(obs);
			par->obstacles[_label].erase(it);
		}
		// cleanup and shrink
		{
			par->obstacles[_label].clear();
			par->obstacles[_label].shrink_to_fit();
			for (auto &sector : child->obstacles)
				sector.shrink_to_fit();
		}
	}

	set_node_status(child);
	if (child->status == UNKNOWN)
	{
		// do not store nodes out of bounds!
		child.reset();
		return nullptr;
	}

	return child;
}

search_node_ptr octree_path_planner::convert_to_graph_node(tree_node_ptr t)
{
	if (t->status != WHITE) {
		return nullptr;
	}

	// preserve id; move search node coordinate to the center of tree node cube
	search_node_ptr s = std::make_shared<search_node>(
		t->id,
		point_3d(
			t->coord.x + t->size / 2,
			t->coord.y + t->size / 2,
			t->coord.z + t->size / 2 
		)
	);

	for (const auto it : t->neighbors)
		if (it->status == WHITE)
			s->neighbor_ids.insert(it->id);

	t->Astar_ref = s;
	return s;
}

void octree_path_planner::construct_octree()
{
	std::cout << "Commencing octree construction..." << std::endl;
	
	int maxdepth;
	m_root = make_root_node(maxdepth);

	int outsiders = 0, blacks = 0, whites = 0, intermediate = 0;
	// global tree node id
	int gid = 1;
	std::vector<tree_node_ptr> queue = {m_root};
	while (!queue.empty())
	{
		tree_node_ptr it = queue.back(); queue.pop_back();
		if (it == nullptr) continue;

		if (it->status == GRAY)
		{
			// create 8 children
			for (int i = 0; i < 8; i++)
			{
				tree_node_ptr it_child = make_child_node(it, (e_cluster_part) i, gid);
				// for now, register nodes out of bounds as nullptrs
				it->children[i] = it_child;

				if (it_child == nullptr) { outsiders++; } 
				
				queue.push_back(it_child);
				gid++;
			}

			// internal search
			it->connect_children();

			// external search
			for (int i = 0; i < 8; i++)	
				external_search(it->children[i]);
			
			intermediate++;
		}
		else if (it->status == WHITE)
		{
			whites++;
		} 
		else if (it->status == BLACK)
		{
			blacks++;
			// free black nodes
			it.reset();
			it = nullptr;
			continue;
		}		
		// else if (m_with_corners > 0 && it->status == WHITE && it->depth < maxdepth - 3)
		// {
		// 	for (int i = 0; i < 8; i++)
		// 	{
		// 		m_tree_nodes.push_back(make_corner_node(it, (e_cluster_part) i, gid, 2));
		// 		it->children[i] = (m_tree_nodes[gid]);
		// 		gid++;
		// 	}
		// 	it->connect_children();
		// 	// external search
		// 	for (int i = 0; i < 8; i++)
		// 		external_search(it->children[i]);
		// }
	}

	if (m_verbose)
	{
		std::cout << "\n------ octree parameters ------" << std::endl;
		std::cout << "Total nodes : " << blacks + whites + intermediate + outsiders << std::endl;
		std::cout << "Intermed.   : " << intermediate << std::endl;
		std::cout << "Dropped     : " << outsiders << std::endl;
		std::cout << "Black nodes : " << blacks << std::endl;
		std::cout << "Leaf nodes  : " << whites + blacks << std::endl;
		std::cout << "Nav cells   : " << whites << std::endl;
		std::cout << "------ octree parameters ------\n" << std::endl;
	}
}

/// define start and goal node IDs by running in-deapth search
void octree_path_planner::set_targets()
{
	m_start_id = 0;
	m_goal_id = 0;

	tree_node_ptr s = m_tree_nodes[0];
	while (s->status != WHITE)
	{
		short xl, yl, zl;
		xl = m_start.x >= s->coord.x + s->size / 2 ? R : L;
		yl = m_start.y >= s->coord.y + s->size / 2 ? F : B;
		zl = m_start.z >= s->coord.z + s->size / 2 ? U : D;
		s = s->children[encode_label(xl, yl, zl)];
		m_start_id = s->id;
	}

	m_search_nodes[m_start_id]->coord = m_start;

	tree_node_ptr g = m_tree_nodes[0];
	while (g->status != WHITE)
	{
		short xl, yl, zl;
		xl = m_goal.x >= g->coord.x + g->size / 2 ? R : L;
		yl = m_goal.y >= g->coord.y + g->size / 2 ? F : B;
		zl = m_goal.z >= g->coord.z + g->size / 2 ? U : D;
		g = g->children[encode_label(xl, yl, zl)];
		m_goal_id = g->id;
	}

	m_search_nodes[m_goal_id]->coord = m_goal;

	if (m_verbose)
	{
		std::cout << "start @ depth " << s->depth 
				  << "; parent: " << s->parent->id 
				  << "; neighbors: " << s->neighbors.size() 
				  << std::endl;
		std::cout << "start treenode located @ " << m_goal_id << "; coords: " 
				  << m_search_nodes[m_goal_id]->coord.x << ", " 
				  << m_search_nodes[m_goal_id]->coord.y << ", " 
				  << m_search_nodes[m_goal_id]->coord.z 
				  << std::endl;

		std::cout << "goal @ depth " << g->depth 
				  << "; parent: " << g->parent->id 
				  << "; neighbors: " << g->neighbors.size() 
				  << std::endl;
		std::cout << "goal treenode located @ " << m_goal_id << "; coords: " 
				  << m_search_nodes[m_goal_id]->coord.x << ", " 
				  << m_search_nodes[m_goal_id]->coord.y << ", " 
				  << m_search_nodes[m_goal_id]->coord.z 
				  << std::endl;
	}
}
