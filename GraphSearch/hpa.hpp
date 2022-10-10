#include "graph_search.hpp"

void octree_path_planner::cluster_dijkstra(std::vector<search_node_ptr> &cluster)
{
	if (cluster.size() <= 1)
		return;
	else
		for (auto a : cluster)
		{
			std::vector<search_node_ptr> buffer; // temp search space
			std::map<search_node_ptr, float> Dijkstra_dist;
			buffer.push_back(a);
			Dijkstra_dist[a] = 0;
			a->opened = true;
			// dijkstra distances calculations

			for (int i = 0; i < buffer.size(); i++)
			{ // auto it = buffer.begin(); it != buffer.end(); it = buffer.begin() + i){
				for (auto n : buffer[i]->neighbor_ids)
				{
					if (m_tree_nodes[n]->cluster_id != m_tree_nodes[buffer[i]->id]->cluster_id)
					{
						continue;
					}
					else if (m_tree_nodes[n]->Astar_ref)
					{
						// if this neighbor is already visited or added, refresh its shortest dist
						if (m_tree_nodes[n]->Astar_ref->opened)
						{
							if (float x = Dijkstra_dist[buffer[i]] + dist(m_tree_nodes[n]->Astar_ref, buffer[i]); x < Dijkstra_dist[m_tree_nodes[n]->Astar_ref])
							{
								Dijkstra_dist[m_tree_nodes[n]->Astar_ref] = x;
								(m_tree_nodes[n]->Astar_ref)->prev_in_path = buffer[i];
							}
						}
						else
						{
							// if new, emplace in buffer and assign shortest dist basing on current node as parent
							buffer.push_back(m_tree_nodes[n]->Astar_ref);
							Dijkstra_dist[m_tree_nodes[n]->Astar_ref] = Dijkstra_dist[buffer[i]] + dist(m_tree_nodes[n]->Astar_ref, buffer[i]);
							m_tree_nodes[n]->Astar_ref->prev_in_path = buffer[i];
							m_tree_nodes[n]->Astar_ref->opened = true;
							// std::cout << "pushed!" << std::endl;
						}
					}
				}
				// std::cout << i << "/" << buffer.size() << ", border - " << cluster.size() << " cluster_id - " << m_tree_nodes[buffer[i]->id]->cluster_id <<std::endl;
			}

			// rewriting distances to a->neighbors
			for (auto an : cluster)
			{
				if (an == a)
					continue;
				if (an->opened)
				{ // auto r = Dijkstra_dist.find(an->octree_ref->Astar_ref); r != Dijkstra_dist.end()){
					a->neighbors[an] = Dijkstra_dist[an->octree_ref->Astar_ref];

					std::vector<int> path;
					m_raw_path.push_back(an);
					search_node_ptr t = an->prev_in_path;
					while (t != a && t)
					{
						m_raw_path.push_back(t);
						t = t->prev_in_path;
					}
					a->paths[an] = path;
				}
			}
			for (int i = 0; i < buffer.size(); i++)
			{
				buffer[i]->opened = false;
				buffer[i]->prev_in_path = nullptr;
			}
		}
	// std::cout << cluster[0]->octree_ref->cluster_id <<std::endl;
	return;
}

void octree_path_planner::insert_node(search_node_ptr &S)
{
	// Dijkstra_plan(a->octree_ref->Astar_ref);
	std::vector<search_node_ptr> buffer; // temp search space
	std::map<search_node_ptr, float> Dijkstra_dist;
	buffer.push_back(S);
	Dijkstra_dist[S] = 0;
	S->opened = true;
	// dijkstra distances calculations

	for (int i = 0; i < buffer.size(); i++)
	{ // auto it = buffer.begin(); it != buffer.end(); it = buffer.begin() + i){
		for (auto n : buffer[i]->neighbor_ids)
		{
			if (m_tree_nodes[n]->cluster_id != m_tree_nodes[buffer[i]->id]->cluster_id)
			{
				continue;
			}
			else if (m_tree_nodes[n]->Astar_ref)
			{
				// if this neighbor is already visited or added, refresh its shortest dist
				if (m_tree_nodes[n]->Astar_ref->opened)
				{ // auto r = std::find(buffer.begin(), buffer.end(), m_tree_nodes[n]->Astar_ref); r != buffer.end()){
					if (float x = Dijkstra_dist[buffer[i]] + dist(m_tree_nodes[n]->Astar_ref, buffer[i]); x < Dijkstra_dist[m_tree_nodes[n]->Astar_ref])
					{
						Dijkstra_dist[m_tree_nodes[n]->Astar_ref] = x;
						(m_tree_nodes[n]->Astar_ref)->prev_in_path = buffer[i];
					}
				}
				else
				{
					// if new, emplace in buffer and assign shortest dist basing on current node as parent
					buffer.push_back(m_tree_nodes[n]->Astar_ref);
					Dijkstra_dist[m_tree_nodes[n]->Astar_ref] = Dijkstra_dist[buffer[i]] + dist(m_tree_nodes[n]->Astar_ref, buffer[i]);
					m_tree_nodes[n]->Astar_ref->prev_in_path = buffer[i];
					m_tree_nodes[n]->Astar_ref->opened = true;
					// std::cout << "pushed!" << std::endl;
				}
			}
		}
		// std::cout << i << "/" << buffer.size() << ", border - " << cluster.size() << " cluster_id - " << m_tree_nodes[buffer[i]->id]->cluster_id <<std::endl;
	}

	// rewriting distances to a->neighbors
	for (auto an_d : S->neighbors)
	{
		auto an = an_d.first;
		if (an == S)
			continue;
		if (an->opened)
		{ // auto r = Dijkstra_dist.find(an->octree_ref->Astar_ref); r != Dijkstra_dist.end()){
			S->neighbors[an] = an->neighbors[S] = Dijkstra_dist[an];

			std::vector<int> path;
			path.push_back(an->id);
			search_node_ptr t = an->prev_in_path;
			while (t && t != S)
			{
				path.push_back(t->id);
				t = t->prev_in_path;
			}
			std::vector<int> repath;
			repath.insert(repath.begin(), path.rbegin(), path.rend());

			S->paths[an] = path;
			an->paths[S] = repath;
		}
	}
	m_search_clusters[S->octree_ref->cluster_id].push_back(S);
	for (int i = 0; i < buffer.size(); i++)
	{
		buffer[i]->opened = false;
		buffer[i]->prev_in_path = nullptr;
	}
}

bool octree_path_planner::Astar_graph_search()
{
	std::vector<search_node_ptr> subgoals;
	search_node_ptr S, G;
	bool S_inserted, G_inserted;
	m_raw_path.clear();
	m_seen_list.assign(m_search_nodes.size(), false);

	// search for start and goal in abstract graph, insert if not found
	if (auto r = std::find(m_search_clusters[m_tree_nodes[m_start_id]->cluster_id].begin(),
						   m_search_clusters[m_tree_nodes[m_start_id]->cluster_id].end(),
						   m_tree_nodes[m_start_id]->Astar_ref);
		r != m_search_clusters[m_tree_nodes[m_start_id]->cluster_id].end())
	{
		S = *r;
		S_inserted = false;
	}
	else
	{
		S = std::make_shared<search_node>(m_start_id, m_start.x, m_start.y, m_start.z, 0,0,0);
		S->octree_ref = m_tree_nodes[m_start_id];
		m_tree_nodes[m_start_id]->Astar_ref = S;
		S->cluster_id = m_tree_nodes[m_start_id]->cluster_id;
		insert_node(S);
		S_inserted = true;
		if (m_verbose)
			std::cout << "start inserted into " << S->id << ", " << S->neighbors.size() << "neighbors, cluster has m_tree_nodes x " << m_search_clusters[S->cluster_id].size() << std::endl;
	}
	if (auto r = std::find(m_search_clusters[m_tree_nodes[m_goal_id]->cluster_id].begin(),
						   m_search_clusters[m_tree_nodes[m_goal_id]->cluster_id].end(),
						   m_tree_nodes[m_goal_id]->Astar_ref);
		r != m_search_clusters[m_tree_nodes[m_goal_id]->cluster_id].end())
	{
		G = *r;
		G_inserted = false;
	}
	else
	{
		G = std::make_shared<search_node>(m_goal_id, m_goal.x, m_goal.y, m_goal.z, 0,0,0);
		G->octree_ref = m_tree_nodes[m_goal_id];
		m_tree_nodes[m_goal_id]->Astar_ref = G;
		G->cluster_id = m_tree_nodes[m_goal_id]->cluster_id;
		insert_node(G);
		G_inserted = true;
		if (m_verbose)
			std::cout << "goal inserted into " << G->id << ", " << G->neighbors.size() << " neighbors, cluster has m_tree_nodes x " << m_search_clusters[G->cluster_id].size() << std::endl;
	}

	fout1.open("HPA_path_desc.txt");
	fout1 << S->coord.x << " " << S->coord.y << " " << S->coord.z << ' ' << S->octree_ref->size << ' ' << "Start" << std::endl;
	fout1 << G->coord.x << " " << G->coord.y << " " << G->coord.z << ' ' << G->octree_ref->size << ' ' << "Goal" << std::endl;

	Astar_priority_queue apq;

	search_node_ptr curr_node = S;

	// Insert start node
	curr_node->heap_key = apq.push(curr_node);
	curr_node->opened = true;
	curr_node->g = 0;
	m_seen_list[curr_node->octree_ref->id] = true;

	int maxExpand = -1;
	auto t1 = std::chrono::steady_clock::now();
	int Abstract_expand = 0;
	bool valid = true;
	while (valid)
	{
		Abstract_expand++;
		// get element with smallest cost
		curr_node = apq.top();
		apq.pop();
		curr_node->closed = true; // Add to closed list
		curr_node->h = dist(m_search_nodes[curr_node->id], m_search_nodes[G->id]);

		fout1 << curr_node->coord.x << " " << curr_node->coord.y << " " << curr_node->coord.z << ' ' << curr_node->octree_ref->size << ' ' << "Expand" << std::endl;

		if (curr_node == G)
		{
			if (m_verbose)
			{
				printf("Goal Reached!!!!!!\n\n");
				std::cout << "HPAstar time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count() << "ms" << std::endl;
			}
			break;
		}

		// printf("expand: %d, %d\n", currNode_ptr->x, currNode_ptr->y);
		std::vector<search_node_ptr> succ_ids;
		std::vector<double> succ_costs;

		//// Get successors
		// if (currNode_ptr->neighbors.empty()) std::cout << "no neighbors!" <<std::endl;

		for (auto [succ, dist] : curr_node->neighbors)
		{

			if (!succ)
			{
				continue;
			}

			if (!m_seen_list[succ->id])
			{
				succ->g = std::numeric_limits<double>::infinity();
				succ->h = 0;
				succ->opened = false;
				succ->closed = false;
				m_seen_list[succ->id] = true;
			}

			succ_ids.push_back(succ);
			succ_costs.push_back(dist);
		}
		// if (succ_ids.empty()) std::cout << "no valid neighbors added!" <<std::endl;

		// Process successors
		for (int s = 0; s < (int)succ_ids.size(); s++)
		{
			// see if we can improve the value of succstate
			search_node_ptr child_ptr = succ_ids[s];
			double tentative_gval = curr_node->g + succ_costs[s];

			if (tentative_gval < child_ptr->g)
			{
				child_ptr->prev_in_path = curr_node; // Assign new parent
				child_ptr->g = tentative_gval;		// Update gval

				// double fval = child_ptr->g + child_ptr->h;

				// if currently in OPEN, update
				if (child_ptr->opened && !child_ptr->closed)
				{
					apq.increase(child_ptr->heap_key); // update heap
				}
				// if currently in CLOSED
				else if (child_ptr->opened && child_ptr->closed)
				{
					printf("ASTAR ERROR!\n");
				}
				else // new node, add to heap
				{
					// printf("add to open set: %d, %d\n", child_ptr->x, child_ptr->y);
					child_ptr->heap_key = apq.push(child_ptr);
					child_ptr->opened = true;
				}
			} //
		}	  // Process successors

		if (maxExpand > 0 && Abstract_expand >= maxExpand)
		{
			if (m_verbose)
				printf("MaxExpandStep [%d] Reached!!!!!!\n\n", maxExpand);
			valid = false;
		}

		if (apq.empty())
		{
			if (m_verbose)
				printf("Priority queue is empty!!!!!!\n\n");
			// std::cout << expand_iteration << std::endl;
			valid = false;
		}
	}

	if (!valid)
	{
		std::cout << "HPA* error!!! expand: " << Abstract_expand << std::endl;
		// subgoals.clear();
		return false;
	}

	if (m_verbose)
	{
		printf("goal g: %f, h: %f!\n", curr_node->g, curr_node->h);
		printf("Expand [%d] m_tree_nodes!\n", Abstract_expand);
	}

	// std::vector<search_node_ptr> octree_path_planner::recoverPath(search_node_ptr node) {
	subgoals.push_back(curr_node->octree_ref->Astar_ref);

	while (curr_node && curr_node != S)
	{
		curr_node = curr_node->prev_in_path;
		subgoals.push_back(curr_node->octree_ref->Astar_ref);
		fout1 << curr_node->coord.x << " " << curr_node->coord.y << " " << curr_node->coord.z << ' ' << curr_node->octree_ref->size << ' ' << "Subgoal" << std::endl;
	}

	int count = 0;
	for (auto substart = subgoals.rbegin(); substart != subgoals.rend(); substart++)
	{

		count++;
		if (count == subgoals.size())
		{
			// std::cout << "done!" << std::endl;
			break;
		}
		auto subgoal = substart + 1;

		if (auto r = m_tree_nodes[(*substart)->id]->Astar_ref->paths.find(m_tree_nodes[(*subgoal)->id]->Astar_ref);
			r != m_tree_nodes[(*substart)->id]->Astar_ref->paths.end())
		{
			// if((*r).second[0] != (*subgoal)->id) std::cout << "wrong subpath?" << std::endl;
			// else
			for (auto p = (*r).second.rbegin(); p != (*r).second.rend(); p++)
			{
				m_raw_path.push_back(m_tree_nodes[*p]->Astar_ref);
				fout1 << m_tree_nodes[*p]->Astar_ref->coord.x << " " << m_tree_nodes[*p]->Astar_ref->coord.y << " " << m_tree_nodes[*p]->Astar_ref->coord.z << ' ' << m_tree_nodes[*p]->size << ' ' << "Path" << std::endl;
			}
			// std::cout << count << std::endl;
		}
		else
			std::cout << "no subpath found!" << std::endl;
	}

	// delete S and G from AbsGraph
	if (S_inserted)
		for (auto it = m_search_clusters[S->cluster_id].begin(); it != m_search_clusters[S->cluster_id].end(); it++)
		{
			if (*it == S)
			{
				m_search_clusters[S->cluster_id].erase(it);
				it--;
			}
			else
			{
				(*it)->neighbors.erase(S);
				(*it)->paths.erase(S);
			}
		}

	if (G_inserted)
		for (auto it = m_search_clusters[G->cluster_id].begin(); it != m_search_clusters[G->cluster_id].end(); it++)
		{
			if (*it == G)
			{
				m_search_clusters[G->cluster_id].erase(it);
				it--;
			}
			else
			{
				(*it)->neighbors.erase(G);
				(*it)->paths.erase(G);
			}
		}

	apq.clear();
	expand_iteration = Abstract_expand;
	fout1.close();
	return true;
}

void octree_path_planner::set_cluster_insides(std::vector<search_node_ptr> &cluster_inside, tree_node_ptr &curr)
{

	std::vector<tree_node_ptr> queue;
	queue.push_back(curr);
	tree_node_ptr current;

	for (int i = 0; i < queue.size(); i++)
	{
		current = queue[i];
		if (current->status == WHITE && current->Astar_ref)
		{
			current->Astar_ref->cluster_id = current->cluster_id;

			search_node_ptr a = convert_to_graph_node(current);
			a->cluster_id = current->cluster_id;
			a->octree_ref = current;
			bool has_couple = false;
			for (auto cn : current->neighbors)
			{
				if (cn->cluster_id != current->cluster_id && cn->status == WHITE)
				{
					a->interedges_ids.push_back(cn->id);
					a->leads_to_clusters.insert(cn->cluster_id);
					if (cn->Astar_ref)
						has_couple = true;
				}
			}
			if (a->leads_to_clusters.size() > 2 || has_couple && ((a->leads_to_clusters.size() == 2 && i % 4 == 0) ||
																  (a->leads_to_clusters.size() == 1 && i % 8 == 0)))
			{
				cluster_inside.push_back(a);
				current->Astar_ref = a;
			}
		}
		else if (current->status == BLACK)
			continue;
		else
			for (auto it : current->children)
				queue.push_back(it);
	}
}

void octree_path_planner::mark_treebranch(int c_id, tree_node_ptr &c)
{
	std::vector<tree_node_ptr> queue;
	queue.push_back(c);
	for (int i = 0; i < queue.size(); i++)
	{
		queue[i]->cluster_id = c_id;
		if (auto x = queue[i]->Astar_ref; x)
			x->cluster_id = c_id;

		if (queue[i]->children.empty())
			continue;
		else
			for (auto ch : queue[i]->children)
				queue.push_back(ch);
	}
}

void octree_path_planner::set_Astar_graph(int cluster_depthlvl)
{
	if (cluster_depthlvl > 0)
	{
		std::vector<tree_node_ptr> clusters;
		std::vector<tree_node_ptr> queue;
		queue.push_back(this->m_tree_nodes[0]);

		// filter cluster m_tree_nodes
		for (const auto & it : queue)
		{
			//tree_node_ptr it = queue[i];
			if (it->depth < cluster_depthlvl)
			{
				if (it->status == WHITE)
				{
					clusters.push_back(it);
				}
				else 
				{
					for (auto n : it->children)
					{
						if (n->status != BLACK) 
							queue.push_back(n);
					}
				}
			}
			else if ((it)->status != BLACK)
			{
				clusters.push_back(it);
			}
		}

		for (auto &it : clusters)
			mark_treebranch(it->id, it);

		queue.clear();
		std::cout << "1. clusters octree m_tree_nodes defined and marked!" << std::endl;
		// AbstractGraph.reserve(clusters.size());

		// extract interedges
		for (auto c = clusters.begin(); c != clusters.end(); c++)
		{
			// current cluster's border m_tree_nodes
			std::vector<search_node_ptr> cluster_inside;
			// recursive descending to white m_tree_nodes inside cluster c
			set_cluster_insides(cluster_inside, *c);
			if (!cluster_inside.empty())
				m_search_clusters[(*c)->cluster_id] = cluster_inside;
		}

		clusters.clear();
	}
	else if (cluster_depthlvl == 0)
	{
		// define straight cluster size
		int ccount = 12;

		int Xsize = dim[0] / ccount;
		int Ysize = dim[1] / ccount;
		int Zsize = dim[2] / ccount;

		// std::vector< std::vector<NodePtr> > clusters_data(pow(ccount+1, 3));
		int id = 0;
		for (int x = 0; x < dim[0]; x++)
			for (int y = 0; y < dim[1]; y++)
				for (int z = 0; z < dim[2]; z++)
				{
					point_3d temp_c(x, y, z);
					if (!is_free(temp_c))
						continue;

					id = coord_to_id(temp_c);
					int cid = x / Xsize + y / Ysize * ccount + z / Zsize * ccount * ccount;
					m_tree_nodes[id]->cluster_id = cid;
					// clusters_data[cid].push_back(m_tree_nodes[id]);
				}
		std::cout << "pass" << std::endl;
		// for (const auto & cluster : clusters_data){

		// if (cluster.size() == 0) continue;
		int i = 0;

		for (const auto &current : m_tree_nodes)
		{
			if (!current)
				continue;
			++i;
			search_node_ptr a = convert_to_graph_node(current);
			a->cluster_id = current->cluster_id;
			a->octree_ref = current;
			bool has_couple = false;
			for (auto cn : current->neighbors)
			{
				if (cn->cluster_id != current->cluster_id && cn->status == WHITE)
				{
					a->interedges_ids.push_back(cn->id);
					a->leads_to_clusters.insert(cn->cluster_id);
					if (cn->Astar_ref)
						has_couple = true;
				}
			}
			if (a->leads_to_clusters.size() > 2 || has_couple && ((a->leads_to_clusters.size() == 2 && i % 10 == 0) ||
																  (a->leads_to_clusters.size() == 1 && i % 20 == 0)))
			{
				m_search_clusters[a->cluster_id].push_back(a);
				current->Astar_ref = a;
			}
			else
				a = nullptr;
		}
		//}
	}

	int nod = 0;
	int cls = 0;
	for (auto cluster = m_search_clusters.begin(); cluster != m_search_clusters.end(); cluster++)
	{
		cls++;
		nod += (*cluster).second.size();
	}
	std::cout << "2. abstract graph m_tree_nodes extracted - " << cls << " clusters, " << nod << " m_tree_nodes total" << std::endl;

	// for each cluster node add interedges to neighbors map
	for (auto &[c_id, cluster] : m_search_clusters)
	{
		cluster_dijkstra(cluster);
		for (auto &it : cluster)
		{
			for (auto n : it->interedges_ids)
				if (!m_tree_nodes[n]->Astar_ref)
					continue;
				else
				{
					it->neighbors[m_tree_nodes[n]->Astar_ref] = dist(m_tree_nodes[n]->Astar_ref, it);
					std::vector<int> path;
					path.push_back(n);
					it->paths[m_tree_nodes[n]->Astar_ref] = path;
				}
		}
	}
	std::cout << "3. interedges defined!" << std::endl;
}
