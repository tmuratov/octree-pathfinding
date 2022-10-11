/**
 * This file is taken from github repository
 * https://github.com/KumarRobotics/jps3d/blob/master/test/read_map.hpp
 * 
*/

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

class MapReader
{
public:
  MapReader(const std::string &file, bool verbose = false)
  {
    try
    {
      YAML::Node config = YAML::LoadFile(file);

      if (!config[0]["start"] || !config[1]["goal"] || !config[2]["origin"] || !config[3]["dim"] || !config[4]["resolution"] || !config[5]["data"])
      {
        printf("Check input format!\n");
        return;
      }

      const std::vector<float> &start = config[0]["start"].as<std::vector<float>>();
      start_.resize(start.size());
      for (unsigned int i = 0; i < start.size(); i++)
        start_[i] = start[i];

      const std::vector<float> &goal = config[1]["goal"].as<std::vector<float>>();
      goal_.resize(goal.size());
      for (unsigned int i = 0; i < goal.size(); i++)
        goal_[i] = goal[i];

      const std::vector<float> &origin_vec = config[2]["origin"].as<std::vector<float>>();
      origin_.resize(origin_vec.size());
      for (unsigned int i = 0; i < origin_vec.size(); i++)
        origin_[i] = origin_vec[i];

      const std::vector<int> &dim_vec = config[3]["dim"].as<std::vector<int>>();
      dim_.resize(dim_vec.size());
      for (unsigned int i = 0; i < dim_vec.size(); i++)
        dim_[i] = dim_vec[i];

      resolution_ = config[4]["resolution"].as<float>();

      const std::vector<int> &data = config[5]["data"].as<std::vector<int>>();
      data_.resize(data.size());
      std::cout << "read " << data.size() << " elements" << std::endl;
      int occupancy = 0;
      for (unsigned int i = 0; i < data.size(); i++)
        data_[i] = data[i] > 0 ? 1 : 0;

      // config.Close();

      exist_ = true;
    }
    catch (YAML::ParserException &e)
    {
      std::cout << e.what() << std::endl;
      exist_ = false;
    }
  }

  bool exist() { return exist_; }
  std::vector<float> origin() { return origin_; }
  std::vector<int> dim() { return dim_; }
  std::vector<float> start() { return start_; }
  std::vector<float> goal() { return goal_; }
  float resolution() { return resolution_; }
  const std::vector<char> &data() { return data_; }

  std::vector<int> CoordToID(std::vector<float> p)
  {
    std::vector<int> r(p.size(), 0);
    for (int i = 0; i < p.size(); i++)
    {
      r[i] = std::round((p[i] - origin_[i]) / resolution_ - 0.5);
    }
    return r;
  }

  ~MapReader() {}

private:
  std::vector<float> start_;
  std::vector<float> goal_;
  std::vector<float> origin_;
  std::vector<int> dim_;
  float resolution_;
  std::vector<char> data_;

  bool exist_ = false;
};
