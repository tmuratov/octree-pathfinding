/**
 * This file is initially taken from github repository
 * https://github.com/KumarRobotics/jps3d/blob/master/test/read_map.hpp
 * 
 * modified to parse raw 3dmap benchmark files
*/

#include <iostream>
#include <fstream>
#include <random>

class MapReader
{
public:
  MapReader(const std::string &file, bool verbose = false)
  {
    try
    {
      std::string ext = file.substr(file.length() - 6);
      if (ext != ".3dmap")
      {
        std::cout << "Awaiting .3dmap file!" <<std::endl;
        std::exit(1);
      }

      std::ifstream map_file;
      map_file.open(file);
      if (!map_file.good())
      {
        std::cout << "Invalid input .3dmap file!" <<std::endl;
        std::exit(1);
      }

      this->dim_.resize(3);
      map_file >> dim_[0] >> dim_[1] >> dim_[2];

      this->data_.clear();
      this->data_.resize(dim_[0]*dim_[1]*dim_[2], 0);

      int counter = 0;
      while (map_file.good())
      {
        int x, y, z = 0;
        map_file >> x >> y >> z;
        int index = x + y * dim_[0] + z* dim_[0]*dim_[1];
        if (data_[index] == 1){
          std::cout << "Point already visited! " << x << y << z << std::endl;
          continue;
        }
        data_[index] = 1;
        counter++;
      }

      std::cout << "Successfully read map data, amount of obstacles: " << counter << std::endl;
      std::cout << "Map dimension: " << dim_[0] << "x" << dim_[1] << "x" << dim_[2] << std::endl;
      std::cout << "Total map size: " << data_.size() << std::endl;
      exist_ = true;

      std::cout << "Setting up random start and goal points..." << std::endl;

      start_ = {
        (float)(std::rand() % dim_[0]),
        (float)(std::rand() % dim_[1]),
        (float)(std::rand() % dim_[2])
      }; 
      std::cout << "Start point: " << start_[0] << "-" << start_[1] << "-" << start_[2] << std::endl;
      goal_ = {
        (float)(std::rand() % dim_[0]),
        (float)(std::rand() % dim_[1]),
        (float)(std::rand() % dim_[2])
      };
      std::cout << "End point: " << goal_[0] << "-" << goal_[1] << "-" << goal_[2] << std::endl;


    }
    catch (std::exception &e)
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
  std::vector<float> origin_ = {0,0,0};
  std::vector<int> dim_;
  float resolution_ = 1;
  std::vector<char> data_;

  bool exist_ = false;
};
