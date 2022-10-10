#include "graph_search.hpp"

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
			children[i]->neighbors.clear();
			children[i]->neighbors.push_back(children[i]->parent);
		}

		this->children[LBD]->neighbors.push_back(children[RBD]);
		this->children[LBD]->neighbors.push_back(children[LFD]);
		this->children[LBD]->neighbors.push_back(children[LBU]);

		this->children[RBD]->neighbors.push_back(children[LBD]);
		this->children[RBD]->neighbors.push_back(children[RFD]);
		this->children[RBD]->neighbors.push_back(children[RBU]);

		this->children[LFD]->neighbors.push_back(children[RFD]);
		this->children[LFD]->neighbors.push_back(children[LBD]);
		this->children[LFD]->neighbors.push_back(children[LFU]);

		this->children[RFD]->neighbors.push_back(children[LFD]);
		this->children[RFD]->neighbors.push_back(children[RBD]);
		this->children[RFD]->neighbors.push_back(children[RFU]);

		this->children[LBU]->neighbors.push_back(children[RBU]);
		this->children[LBU]->neighbors.push_back(children[LFU]);
		this->children[LBU]->neighbors.push_back(children[LBD]);

		this->children[RBU]->neighbors.push_back(children[LBU]);
		this->children[RBU]->neighbors.push_back(children[RFU]);
		this->children[RBU]->neighbors.push_back(children[RBD]);

		this->children[LFU]->neighbors.push_back(children[RFU]);
		this->children[LFU]->neighbors.push_back(children[LBU]);
		this->children[LFU]->neighbors.push_back(children[LFD]);

		this->children[RFU]->neighbors.push_back(children[LFU]);
		this->children[RFU]->neighbors.push_back(children[RBU]);
		this->children[RFU]->neighbors.push_back(children[RFD]);
	}
}

int tree_node::get_status() const
{
	return status;
}

void octree_path_planner::external_search(tree_node_ptr node)
{
	for (auto it = node->parent->neighbors.begin(); it != node->parent->neighbors.end(); it++)
	{
		bool are_neighbors = node->access_neighboring(*it);
		if (are_neighbors)
		{
			node->neighbors.push_back(*it);
			(*it)->neighbors.push_back(node);
		}
	}
}

bool tree_node::access_neighboring(tree_node_ptr n)
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
		if (node->coord.x >= dim[0] || node->coord.y >= dim[1] || node->coord.z >= dim[3])
			node->status = e_state_desc::UNKNOWN; // node out of map range --- unknown
		else
			node->status = e_state_desc::WHITE; // node is free to move through
	}
	else if (node->occupancy == node->capacity)
		node->status = e_state_desc::BLACK; // node is completely blocked, no need to divide further
	else
		node->status = e_state_desc::GRAY; // node contains obstacles, need to be further divided,
}

tree_node_ptr octree_path_planner::make_root_node(int &maxdepth)
{
	// define tree size: get closest ceiled pow2 to max dimension of raw map
	int max_dim = 0; 
	for (const int & dim_i : dim)
		if (dim_i > max_dim) 
			max_dim = dim_i;
 	
	int treeside = 2;
	maxdepth = 1;
	while (treeside < max_dim)
	{
		treeside *= 2;
		maxdepth++;
	}

	tree_node_ptr root = std::make_shared<tree_node>(0, treeside, 0, 0, 0, nullptr, NONE);
	root->capacity = std::pow(treeside, 3); // size*size*size;
	root->status = e_state_desc::UNSET;
	root->depth = 0;
	root->occupancy = 0;

	int xl = L, yl = B, zl = D; //(0,0,0) by def
	for (int x = 0; x < dim[0]; x++)
		for (int y = 0; y < dim[1]; y++)
			for (int z = 0; z < dim[2]; z++)
			{
				point_3d coord(x, y, z);
				if (is_occupied(coord))
				{
					xl = x >= treeside / 2 ? R : L;
					yl = y >= treeside / 2 ? F : B;
					zl = z >= treeside / 2 ? U : D;
					root->occupancy++;
					root->obstacles[encode_label(xl, yl, zl)].push_back(coord);
				}
			}

	set_node_status(root);

	return root;
}

tree_node_ptr octree_path_planner::make_child_node(tree_node_ptr p, int _label, int _id)
{
	int _size = p->size / 2;
	e_cluster_part e_label = (e_cluster_part)_label;
	point_3d tmp_label = decode_label(e_label);
	point_3d C(p->coord.x, p->coord.y, p->coord.z);

	if (tmp_label.x == R)
		C.x += _size;
	if (tmp_label.y == F)
		C.y += _size;
	if (tmp_label.z == U)
		C.z += _size;

	tree_node_ptr child = std::make_shared<tree_node>(_id, _size, C.x, C.y, C.z, p, e_label);

	child->capacity = _size * _size * _size;
	child->occupancy = p->obstacles[_label].size();
	child->depth = p->depth + 1;

	child->status = e_state_desc::UNSET;

	if (child->occupancy > 0)
	{
		int xl, yl, zl;

		for (auto it = p->obstacles[_label].begin(); it != p->obstacles[_label].end();)
		{

			xl = (*it).x >= C.x + _size / 2 ? R : L;
			yl = (*it).y >= C.y + _size / 2 ? F : B;
			zl = (*it).z >= C.z + _size / 2 ? U : D;
			child->obstacles[xl + yl + zl].push_back(*it);
			p->obstacles[_label].erase(it);
		}
	}
	set_node_status(child);
	
	child->neighbors.reserve(30);
	return child;
}

tree_node_ptr octree_path_planner::make_corner_node(tree_node_ptr p, int _label, int _id, int corner_size)
{
	int _size = corner_size; // p->size/pow(2, maxdepth - p->depth);
	e_cluster_part e_label = (e_cluster_part)_label;
	point_3d tmp = decode_label(e_label);
	point_3d C(p->coord.x, p->coord.y, p->coord.z);
	if (tmp.x == R)
		C.x += _size;
	else
		C.x += p->size - _size;
	if (tmp.y == F)
		C.y += _size;
	else
		C.y += p->size - _size;
	if (tmp.z == U)
		C.z += _size;
	else
		C.z += p->size - _size;

	tree_node_ptr corner = std::make_shared<tree_node>(_id, _size, C.x, C.y, C.z, p, e_label);

	// corner->capacity = _size*_size*_size;
	// corner->occupancy = p->obstacles[_label].size();
	corner->depth = p->depth + 4;

	corner->status = WHITE;
	// if (corner->GetStatus() == BLACK)
	// std::cout << "corner info: id(" << _id << "); parent(" << p->id << "); depthlvl(" << corner.depth << "); status(" << corner.status << "); label(" << _label << "); capacity(" << corner.capacity << "); occupancy(" << corner.occupancy <<"); neighbors - " << corner.neighbors.size() << std::endl;

	corner->neighbors.reserve(15);

	return corner;
}

search_node_ptr octree_path_planner::convert_to_graph_node(tree_node_ptr t)
{
	search_node_ptr s = std::make_shared<search_node>(t->id,
										t->coord.x + t->size / 2,
										t->coord.y + t->size / 2,
										t->coord.z + t->size / 2,
										0,0,0
									);
	s->dx = s->dy = s->dz = 0;

	for (const auto it : t->neighbors)
		if (it->get_status() == WHITE)
			s->neighbor_ids.insert(it->id);
	t->Astar_ref = s;
	return s;
}

void octree_path_planner::construct_octree()
{
	std::cout << "commencing octree construction" << std::endl;
	// reserve some space in the vector (statistically, octree usually holds ~50 times less nodes that unigrid)
	m_tree_nodes.clear();
	m_tree_nodes.reserve(dim[0] * dim[1] * dim[2] / 50);
	
	int maxdepth;
	auto root = make_root_node(maxdepth);
	
	m_tree_nodes.push_back(root);
	std::cout << "root info: id(" << root->id << "); maxdepth(" << maxdepth 
			  << "); status(" << root->status << "); size(" << root->size << "); capacity(" 
			  << root->capacity << "); occupancy(" << root->occupancy << ")" 
			  << std::endl;

	// global tree node id
	int gid = 1;
	for (int it = 0; it < m_tree_nodes.size(); it++)
	{
		if (m_tree_nodes[it]->status == GRAY)
		{
			// create 8 children
			for (int i = 0; i < 8; i++)
			{
				m_tree_nodes.push_back(make_child_node(m_tree_nodes[it], i, gid));
				m_tree_nodes[it]->children.push_back(m_tree_nodes[gid]);
				// m_tree_nodes[it].neighbors.push_back(&m_tree_nodes[id]);
				gid++;
			}

			// internal search
			m_tree_nodes[it]->connect_children();

			// external search
			for (int i = 0; i < 8; i++)
			{
				external_search(m_tree_nodes[it]->children[i]);
			}
		}

		// append octree with corner nodes 
		// else if (m_tree_nodes[it]->status == WHITE && m_tree_nodes[it]->depth < maxdepth - 3)
		// {
		// 	for (int i = 0; i < 8; i++)
		// 	{
		// 		m_tree_nodes.push_back(make_corner_node(m_tree_nodes[it], i, gid, 2));
		// 		m_tree_nodes[it]->children.push_back(m_tree_nodes[id]);
		// 		id++;
		// 	}
		// 	m_tree_nodes[it]->connect_children();
		// 	// external search
		// 	for (int i = 0; i < 8; i++)
		// 		external_search(m_tree_nodes[it]->children[i]);
		// 		//m_tree_nodes[it]->children[i]->external_search(m_tree_nodes);
		// }
	}
	// int whites = 0;
	// m_search_nodes.clear();
	// for (auto it = m_tree_nodes.begin(); it != m_tree_nodes.end(); it++)
	// {
	// 		if ((*it)->get_status() == WHITE)
	// 			whites++;
	// 		m_search_nodes[(*it)->id] = convert_to_graph_node(*it);
	// }
	// std::cout << "reduced to " << whites << " white m_tree_nodes" << std::endl;
}

/// define start and goal node IDs by running in-deapth search
void octree_path_planner::set_targets()
{
	m_start_id = 0;
	m_goal_id = 0;

	tree_node_ptr s = m_tree_nodes[0];
	while (s->get_status() != WHITE)
	{
		int xl, yl, zl;
		xl = m_start.x >= s->coord.x + s->size / 2 ? R : L;
		yl = m_start.y >= s->coord.y + s->size / 2 ? F : B;
		zl = m_start.z >= s->coord.z + s->size / 2 ? U : D;
		s = s->children[xl + yl + zl];
		m_start_id = s->id;
	}

	m_search_nodes[m_start_id]->coord = m_start;

	tree_node_ptr g = m_tree_nodes[0];
	while (g->get_status() != WHITE)
	{
		int xl, yl, zl;
		xl = m_goal.x >= g->coord.x + g->size / 2 ? R : L;
		yl = m_goal.y >= g->coord.y + g->size / 2 ? F : B;
		zl = m_goal.z >= g->coord.z + g->size / 2 ? U : D;
		g = g->children[xl + yl + zl];
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
