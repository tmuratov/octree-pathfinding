
#include "GraphSearch/octree.hpp"
#include "GraphSearch/hpa.hpp"
#include "read_map.hxx"

class MapReader;

int main(int argc, char **argv)
{
	if (argc < 2)
	{
		std::cout << "Provide a map filename in parameters!" << std::endl;
		return 1;
	}

	MapReader reader(argv[1]);
	if (!reader.exist())
	{
		std::cout << "Could not parse map!" << std::endl;
		return 2;
	}

	std::vector<int> start = reader.CoordToID(reader.start());
	std::vector<int> goal = reader.CoordToID(reader.goal());
	std::vector<int> dim = reader.dim();

	auto origin = reader.origin();
	auto resolution = reader.resolution();
	const std::vector<char> &data = reader.data();

	short int HPA_cluster_depth_level = 0;
	std::cout << "set HPA cluster level: ";
	std::cin >> HPA_cluster_depth_level;

	octree_path_planner gs(data.data(), dim[0], dim[1], dim[2], HPA_cluster_depth_level);

	// scenario: a set of pairs (start, goal)
	// results: amount of nodes in path, length, calculation time
	std::ifstream scenario;
	std::ofstream results;

	if (argc > 2)
	{
		scenario.open(argv[2]);
		results.open("octree_results.txt");

		// header
		results << "expansion \t distance \t diff (actual dist) \t time" << std::endl;

		std::cout << "running series: \n\n\n"
				  << std::endl;
		float scen_dist, scen_ratio;

		int counter = 0;
		auto p1 = std::chrono::steady_clock::now();
		while (scenario.good())
		{
			scenario >> start[0] >> start[1] >> start[2] >> goal[0] >> goal[1] >> goal[2] >> scen_dist >> scen_ratio;

			auto pp1 = std::chrono::steady_clock::now();
			if (!gs.plan(start[0], start[1], start[2], goal[0], goal[1], goal[2], -1))
				continue;
			auto pp2 = std::chrono::steady_clock::now();

			auto path = gs.get_path();
			float total_dist = 0.0f;
			for (auto it = path.begin(); it != path.end() - 1; it++)
			{
				total_dist += dist((*(it + 1)), (*it));
			}

			auto this_time = std::chrono::duration_cast<std::chrono::milliseconds>(pp2 - pp1).count();

			results << gs.expand_iteration << "\t" << total_dist << "\t" << scen_dist - total_dist << "\t" << this_time << std::endl;
			counter++;

			if (counter % 100 == 0)
				std::cout << "iteration #" << counter << std::endl;
		}
		auto p2 = std::chrono::steady_clock::now();
		auto total = std::chrono::duration_cast<std::chrono::milliseconds>(p2 - p1).count();
		results << total << std::endl;
		std::cout << "total time series: " << total << " ms" << std::endl;
	}
	scenario.close();
	results.close();

	std::cout << "start \t goal \t dim" << std::endl;
	for (int i = 0; i < 3; i++)
	{
		std::cout << start[i] << "\t" << goal[i] << "\t" << dim[i] << std::endl;
	}
	gs.plan(start[0], start[1], start[2], 
			goal[0], goal[1], goal[2]
			);

	auto path = gs.get_path();
	float total_dist = 0.0f;
	for (auto it = path.begin(); it != path.end() - 1; it++)
	{
		total_dist += dist((*(it + 1)), *it);
	}

	std::cout << "path:" << std::endl;
	for (auto it = path.begin(); it != path.end(); it++)
	{
		std::cout << (*it)->coord.x << "\t" << (*it)->coord.y << "\t" << (*it)->coord.z << std::endl;
	}

	std::cout << "path distance: " << total_dist << std::endl;

	return 0;
}
