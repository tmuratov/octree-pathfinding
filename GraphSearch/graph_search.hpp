#ifndef GRAPH_SEARCH_HPP
#define GRAPH_SEARCH_HPP

#include "graph_search.h"
#include <cmath>
#include <chrono>
#include <iostream>

inline e_cluster_part encode_label(short xl, short yl, short zl)
{
    return e_cluster_part(xl + yl + zl);
}

point_3d decode_label(e_cluster_part label)
{
    int x = 0, y = 0, z = 0;
    int temp = (int)label;

    if (temp >= 4)
    {
        temp -= 4;
        z = U;
    }

    if (temp >= 2)
    {
        temp -= 2;
        y = F;
    }

    x = temp; // L or R

    return point_3d(x, y, z);
}

inline double dist(tree_node_ptr C1, tree_node_ptr C2)
{
    return std::sqrt(std::pow(C2->coord.x - C1->coord.x, 2) +
                     std::pow(C2->coord.y - C1->coord.y, 2) +
                     std::pow(C2->coord.z - C1->coord.z, 2));
}
inline double dist(search_node_ptr C1, search_node_ptr C2)
{
    return std::sqrt(std::pow(C2->coord.x - C1->coord.x, 2) +
                     std::pow(C2->coord.y - C1->coord.y, 2) +
                     std::pow(C2->coord.z - C1->coord.z, 2));
}
inline double dist(point_3d C1, point_3d C2)
{
    return std::sqrt(std::pow(C2.x - C1.x, 2) +
                     std::pow(C2.y - C1.y, 2) +
                     std::pow(C2.z - C1.z, 2));
}

octree_path_planner::octree_path_planner(
    const char *char_map,
    int xN, int yN, int zN,
    short int recurse_lvl,
    double eps,
    bool verbose) 
    : m_map_data(char_map), dim({xN, yN, zN}), m_eps(eps), m_verbose(verbose)
{
    auto t1 = std::chrono::steady_clock::now();
    construct_octree();
    std::cout << "tree construction time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count() << "ms" << std::endl;

    std::cout << "constructing Search graph, recurse level " << recurse_lvl << std::endl;
    set_Astar_graph(recurse_lvl);
    std::cout << "octree + abstract graph construction time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count() << "ms" << std::endl;

    if (m_verbose)
    {
        std::ofstream fout("graph-info.txt");
        std::vector<tree_node_ptr> queue;
        queue.push_back(m_tree_nodes[0]);

        for (int i = 0; i < queue.size(); i++)
        {
            if (queue[i]->status == GRAY)
                queue.insert(queue.end(), queue[i]->children.begin(), queue[i]->children.end());

            else if (queue[i]->status == BLACK)
                fout << " " << queue[i]->Astar_ref->coord.x << " " << queue[i]->Astar_ref->coord.y << " " << queue[i]->Astar_ref->coord.z << " " << queue[i]->size << " " << queue[i]->status << " " << std::endl;

            else if (queue[i]->status == WHITE)
            {
                if (!queue[i]->children.empty())
                    queue.insert(queue.end(), queue[i]->children.begin(), queue[i]->children.end());

                if (queue[i]->Astar_ref)
                    fout << " " << queue[i]->Astar_ref->coord.x << " " << queue[i]->Astar_ref->coord.y << " " << queue[i]->Astar_ref->coord.z << " " << queue[i]->size << " " << 2 << " " << std::endl;
                else
                    fout << " " << queue[i]->Astar_ref->coord.x << " " << queue[i]->Astar_ref->coord.y << " " << queue[i]->Astar_ref->coord.z << " " << queue[i]->size << " " << queue[i]->status << " " << std::endl;
            }
        }
        fout.close();
    }
    // exit(0);
}

inline int octree_path_planner::coord_to_id(point_3d v) const
{
    return v.x + v.y * dim[0] + v.z * dim[0] * dim[1];
}

inline bool octree_path_planner::is_free(point_3d v) const
{
    return v.x >= 0 && v.x < dim[0] 
        && v.y >= 0 && v.y < dim[1] 
        && v.z >= 0 && v.z < dim[2] 
        && m_map_data[coord_to_id(v)] == 0;
}

inline bool octree_path_planner::is_occupied(point_3d v) const
{
    return v.x >= 0 && v.x < dim[0] 
        && v.y >= 0 && v.y < dim[1] 
        && v.z >= 0 && v.z < dim[2] 
        && m_map_data[coord_to_id(v)] > 0;
}

inline double octree_path_planner::get_heuristic_weight(point_3d v) const
{
    return m_eps * dist(v, m_goal);
}

bool octree_path_planner::plan(int xs, int ys, int zs, int xg, int yg, int zg, int max_expand)
{
    m_raw_path.clear();
    m_seen_list.assign(m_tree_nodes.size(), false);
    m_search_front.clear();

    m_start = point_3d(xs, ys, zs);
    m_goal = point_3d(xg, yg, zg);
    set_targets();

    search_node_ptr current_node = m_tree_nodes[m_start_id]->Astar_ref;

    current_node->g = 0;
    current_node->h = 0;
    current_node->h += get_heuristic_weight(m_start);
    bool result = Astar_graph_search();

    current_node = m_search_nodes[m_start_id];
    current_node->g = 0;
    current_node->h = 0;
    current_node->h += get_heuristic_weight(m_start);

    return result && plan(current_node, max_expand);
}

bool octree_path_planner::plan(search_node_ptr &current_node, int max_expand)
{
    // Insert start node
    current_node->heap_key = m_search_front.push(current_node);
    current_node->opened = true;
    // m_search_front[current_node->id] = current_node;
    m_seen_list[current_node->id] = true;

    fout2.open("octree_path_desc.txt");
    fout2 << m_tree_nodes[m_start_id]->coord.x << ' ' << m_tree_nodes[m_start_id]->coord.y << ' ' << m_tree_nodes[m_start_id]->coord.z << ' ' << m_tree_nodes[m_start_id]->size << ' ' << "Start" << std::endl;
    fout2 << m_tree_nodes[m_goal_id]->coord.x << ' ' << m_tree_nodes[m_goal_id]->coord.y << ' ' << m_tree_nodes[m_goal_id]->coord.z << ' ' << m_tree_nodes[m_goal_id]->size << ' ' << "Goal" << std::endl;

    auto t1 = std::chrono::steady_clock::now();
    expand_iteration = 0;
    while (true)
    {
        expand_iteration++;
        // get element with smallest cost
        current_node = m_search_front.top();
        m_search_front.pop();
        current_node->closed = true; // Add to closed list
        fout2 << current_node->coord.x << " " << current_node->coord.y << " " << current_node->coord.z << ' ' << m_tree_nodes[current_node->id]->size << ' ' << "Expand" << std::endl;

        if (current_node->id == m_goal_id)
        {
            std::cout << "Path planning done!" << std::endl;
            std::cout << "Astar time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count() << "ms" << std::endl;
            std::cout << "Expanded " << expand_iteration << std::endl;

            break;
        }

        std::vector<int> succ_ids;
        std::map<int,double> succ_costs;
        // Get successors

        get_successors(current_node, succ_costs);

        // Process successors
        for (const std::pair<int, double> &id_cost : succ_costs)
        {
            // see if we can improve the value of succstate
            search_node_ptr &child_ptr = m_search_nodes[id_cost.first];

            double tentative_gval = current_node->g + id_cost.second;
            if (tentative_gval < child_ptr->g)
            {
                child_ptr->prev_in_path = current_node; // Assign new parent
                child_ptr->g = tentative_gval;          // Update gval

                // double fval = child_ptr->g + child_ptr->h;

                // if currently in OPEN, update
                if (child_ptr->opened && !child_ptr->closed)
                {
                    m_search_front.increase(child_ptr->heap_key); // update heap
                }
                // if currently in CLOSED
                else if (child_ptr->opened && child_ptr->closed)
                {
                    std::cout << "ASTAR ERROR!" << std::endl;
                }
                else // new node, add to heap
                {
                    // printf("add to open set: %d, %d\n", child_ptr->x, child_ptr->y);
                    child_ptr->heap_key = m_search_front.push(child_ptr);
                    child_ptr->opened = true;
                }
            } //
        }     // Process successors

        if (max_expand > 0 && expand_iteration >= max_expand)
        {
            if (m_verbose)
                std::cout << "Max expansion level reached: " << expand_iteration << std::endl;
            return false;
        }

        if (m_search_front.empty())
        {
            if (m_verbose)
                std::cout << "Priority queue is empty!" << std::endl;
            std::cout << "After " << expand_iteration << " iterations" << std::endl;
            return false;
        }
    }

    if (m_verbose)
    {
        std::cout << "Goal g-val: " << current_node->g << ", h: " << current_node->h << std::endl;
        std::cout << "Expansion: " << expand_iteration << " nodes!" << std::endl;
    }

    m_raw_path = recover_path(current_node);
    for (const search_node_ptr p : m_raw_path)
    {
        fout2 << p->coord.x << " " << p->coord.y << " " << p->coord.z << ' ' << m_tree_nodes[p->id]->size << ' ' << "Path" << std::endl;
    }
    fout2.close();
    return true;
}

std::vector<search_node_ptr> octree_path_planner::recover_path(search_node_ptr node)
{
    std::vector<search_node_ptr> path;
    do
    {
        path.push_back(node);
        node = node->prev_in_path;
    }
    while (node && node->id != m_start_id);

    return path;
}

// todo rework
void octree_path_planner::get_successors(const search_node_ptr &curr, std::map<int, double> &succ_costs)
{
    for (const auto &new_id : curr->neighbor_ids)
    {
        if (m_tree_nodes[m_start_id]->cluster_id != m_tree_nodes[new_id]->cluster_id)
            // int new_id = coordToId(new_x, new_y, new_z);
            if (!m_seen_list[new_id])
            {
                m_search_nodes[new_id]->g = std::numeric_limits<double>::infinity();
                m_search_nodes[new_id]->opened = false;
                m_search_nodes[new_id]->closed = false;
                m_seen_list[new_id] = true;
                m_search_nodes[new_id]->h = get_heuristic_weight(m_search_nodes[new_id]->coord);
            }
        std::array<float, 3> d;
        d[0] = m_search_nodes[new_id]->coord.x - curr->coord.x;
        d[1] = m_search_nodes[new_id]->coord.y - curr->coord.y;
        d[2] = m_search_nodes[new_id]->coord.z - curr->coord.z;
        succ_costs[new_id] = (std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
    }
}

std::vector<search_node_ptr> octree_path_planner::get_path() const
{
    return m_raw_path;
}

std::vector<search_node_ptr> octree_path_planner::get_open_set() const
{
    std::vector<search_node_ptr> open;
    for (const auto &it : m_search_nodes)
    {
        const search_node_ptr & node = it.second; 
        if (node && node->opened && !node->closed)
            open.push_back(node);
    }
    return open;
}

std::vector<search_node_ptr> octree_path_planner::get_close_set() const
{
    std::vector<search_node_ptr> close;
    for (const auto &it : m_search_nodes)
    {
        const search_node_ptr & node = it.second; 
        if (node && node->closed)
            close.push_back(node);
    }
    return close;
}

std::vector<search_node_ptr> octree_path_planner::get_all_set() const
{
    std::vector<search_node_ptr> all;
    for (const auto &it : m_search_nodes)
    {
        if (it.second)
            all.push_back(it.second);
    }
    return all;
}

#endif