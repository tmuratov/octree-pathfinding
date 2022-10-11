#include "GraphSearch/octree.hpp"
#include "GraphSearch/hpa.hpp"
#include "MapUtils/read_3dmap_raw.hpp"

class MapReader;

void print_help_msg(int exitcode = 0)
{
	std::cout << "Usage: hpa.exe <3dmap_file> [options].\n" <<
					  	 "3dmap_file - mandatory argument, a map file to be parsed. \n" <<
						 "options:\n" <<
						 "--script=<file> : a path to script without whitespaces - text file containing series of (goal, start) points to successive search (i.e. benchmark instruction). \n" <<
						 "--verbose, -v : verbosity switch, enables additional runtime information. \n" <<
						 "--help, -h : display this message and exit; ignores other arguments. \n" <<
						 std::endl;
	std::exit(exitcode);
}
bool parse_args(int argc, char** argv, std::string &file, std::string &script, bool &verbosity)
{
	if (argc < 2)
	{
		std::cout << "Provide a map filename in parameters!" << std::endl;
		print_help_msg(1);
	}
	file = argv[1];

	for (int i = 2; i < argc; i++)
	{
		std::string str_arg(argv[i]);
		if (str_arg == "--verbose" || str_arg == "-v")
		{
			verbosity = true;
		}
		// no whitespace allowed in file path
		else if (str_arg.compare(0,9, "--script=") == 0)
		{
			script = str_arg.substr(10);
			// trim quotes
			script.erase(script.find_last_not_of("\"\'")+1);
			script.erase(0, script.find_first_not_of("\"\'")+1);
		}
		else if (str_arg == "--help" || str_arg == "-h")
		{
			print_help_msg();
		}
		else return false;
	}
	return true;
}

int main(int argc, char **argv)
{
	std::string file = "";
	std::string script = "";
	bool verbosity = false;

	if( !parse_args(argc, argv, file, script, verbosity))
	{
		std::cout << "Could not parse cmd arguments!" << std::endl;
		print_help_msg(1);
	}

	MapReader reader(file);
	if (!reader.exist())
	{
		std::cout << "Could not parse map!" << std::endl;
		print_help_msg(2);
	}

	std::vector<int> start = reader.CoordToID(reader.start());
	std::vector<int> goal = reader.CoordToID(reader.goal());
	std::vector<int> dim = reader.dim();

	auto origin = reader.origin();
	// auto resolution = reader.resolution();
	const std::vector<char> &data = reader.data();

	short int HPA_cluster_depth_level = 3;
	//std::cout << "set HPA cluster level: ";
	//std::cin >> HPA_cluster_depth_level;

	octree_path_planner gs(data.data(), dim[0], dim[1], dim[2], HPA_cluster_depth_level, 1.0, verbosity);

	// scenario: a set of pairs (start, goal)
	// results: amount of nodes in path, length, calculation time
	std::ifstream scenario;
	std::ofstream results;

	if (!script.empty())
	{
		scenario.open(script);
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

	std::cout << "Path:" << std::endl;
	for (auto it = path.begin(); it != path.end(); it++)
	{
		std::cout << (*it)->coord.x << "\t" << (*it)->coord.y << "\t" << (*it)->coord.z << std::endl;
	}

	std::cout << "path distance: " << total_dist << std::endl;

	return 0;
}
