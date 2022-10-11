/**
 * this program converts .3dmap file to YAML format so that they can be used with jps3d planner
 * 
 * .3dmap files are available here: https://movingai.com/benchmarks/warframe/index.html 
 * jps3d planner available here:    https://github.com/KumarRobotics/jps3d
*/

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
    std::ifstream input;
    bool read = false;
    if (argc > 1)
    {
        input.open(argv[1]);
        read = true;
        if (!input.good())
        {
            std::cout << "Failed to parse file: " << argv[1] << std::endl;
            std:: cout << "Exit with code 1" << std::endl;
            exit(1);
        }
    }
    else std::cout << "No input file; creating sample map..." << std::endl;

    // Set start & goal
    std::vector<double> start{0.2, 0.3, 0.2}; // 2, 3, 2
    std::vector<double> goal{2.7, 2.9, 2.1};  //
    // Create a map
    std::vector<double> origin{0, 0, 0};      // set origin at (0, 0, 0)
    std::vector<int> dim(3, 32);              // set the number of cells in each dimension as 20, 10, 1
    double res = 1;                           // coordinate scaling - for purpose of compatibility with JPS planner
    std::vector<int> data;                    // occupancy data, the subscript follows: id = x + dim.x * y + dim.x * dim.y * z;
    data.resize(dim[0] * dim[1] * dim[2], 0); // initialize as free space, empty cell has 0 occupancy

    if (read)
    {
        input >> dim[0] >> dim[1] >> dim[2];
        data.resize(dim[0] * dim[1] * dim[2], 0);
        for (int i = 0; i < 3; i++)
            goal[i] = (dim[i] - 10 * i) * res;

        int x, y, z, id;
        while (input.good())
        {
            input >> x >> y >> z;
            id = x + dim[0] * y + dim[0] * dim[1] * z;
            data[id] = 100;
        }
    }
    else
    {
        for (int x = 10; x < 32; x++)
        {
            for (int y = 25; y < 32; y++)
            {
                for (int z = 30; z < 32; z++)
                {
                    int id = x + dim[0] * y + dim[0] * dim[1] * z;
                    data[id] = 100;
                }
            }
        }

        for (int x = 20; x < 25; x++)
        {
            for (int y = 15; y < 25; y++)
            {
                for (int z = 5; z < 25; z++)
                {
                    int id = x + dim[0] * y + dim[0] * dim[1] * z;
                    data[id] = 100;
                }
            }
        }

        for (int x = 1; x < 25; x++)
        {
            for (int y = 5; y < 20; y++)
            {
                for (int z = 5; z < 16; z++)
                {
                    int id = x + dim[0] * y + dim[0] * dim[1] * z;
                    data[id] = 100;
                }
            }
        }

        for (int x = 0; x < 6; x++)
        {
            for (int y = 15; y < 30; y++)
            {
                for (int z = 5; z < 30; z++)
                {
                    int id = x + dim[0] * y + dim[0] * dim[1] * z;
                    data[id] = 100;
                }
            }
        }
    }

    YAML::Emitter out;
    out << YAML::BeginSeq;
    // Encode start coordinate
    out << YAML::BeginMap;
    out << YAML::Key << "start" << YAML::Value << YAML::Flow << start;
    out << YAML::EndMap;
    // Encode goal coordinate
    out << YAML::BeginMap;
    out << YAML::Key << "goal" << YAML::Value << YAML::Flow << goal;
    out << YAML::EndMap;
    // Encode origin coordinate
    out << YAML::BeginMap;
    out << YAML::Key << "origin" << YAML::Value << YAML::Flow << origin;
    out << YAML::EndMap;
    // Encode dimension as number of cells
    out << YAML::BeginMap;
    out << YAML::Key << "dim" << YAML::Value << YAML::Flow << dim;
    out << YAML::EndMap;
    // Encode resolution
    out << YAML::BeginMap;
    out << YAML::Key << "resolution" << YAML::Value << res;
    out << YAML::EndMap;
    // Encode occupancy
    out << YAML::BeginMap;
    out << YAML::Key << "data" << YAML::Value << YAML::Flow << data;
    out << YAML::EndMap;

    out << YAML::EndSeq;
    std::cout << "Here is the example map:\n"
              << out.c_str() << std::endl;

    std::ofstream file;
    if (read)
        file.open("complex_WF.yaml");
    else
        file.open("simple.yaml");
    file << out.c_str();
    file.close();

    return 0;
}
