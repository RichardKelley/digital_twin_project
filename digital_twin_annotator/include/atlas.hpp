#ifndef ATLAS_HPP
#define ATLAS_HPP

#include <iostream>
#include <vector>
#include <utility>
#include <limits>

#include <fmt/core.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <map_tile.hpp>
#include <point2.hpp>
#include <landmark.hpp>

#include <yaml-cpp/yaml.h>

namespace digital_twin {
  
  template<typename PointT>
  class Atlas {
    using TileVector = std::vector<MapTile<PointT>>;
      
    std::vector<TileVector> tiles;
    
    int num_x_tiles;
    int num_y_tiles;

    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();

    float x_tile_size = 0.0;
    float y_tile_size = 0.0;

    pcl::PointCloud<PointT>::ConstPtr whole_cloud;
    int extraction_ctr = 0;

    std::vector<Landmark<PointT>> landmarks;
    
  public:

    int get_num_landmarks() const {
      return landmarks.size();
    }

    Atlas(const pcl::PointCloud<PointT>::ConstPtr cloud, int x_tiles, int y_tiles)
      : num_x_tiles{x_tiles}
      , num_y_tiles{y_tiles} {

      tiles.resize(x_tiles);
      for (auto& row : tiles) {
	row.resize(y_tiles);
      }

      whole_cloud = cloud;
      this->initialize_tiles(cloud);
    }

    /*
      Precondition: requires that x_tile_width and y_tile_width are
      properly set.
    */
    std::pair<int,int> get_tile_indices(const PointT& pt) {
      float x_idx_float = (pt.x - min_x) / x_tile_size;
      float y_idx_float = (pt.y - min_y) / y_tile_size;
      
      return std::make_pair(static_cast<int>(x_idx_float), static_cast<int>(y_idx_float));
    }
    
    void initialize_tiles(const pcl::PointCloud<PointT>::ConstPtr cloud) {
      std::cerr << "initialize_tiles" << std::endl;

      // get the bounding rectangle for cloud.
      for (const auto& pt : cloud->points) {
	if (pt.x < min_x) { min_x = pt.x; }
	if (pt.x > max_x) { max_x = pt.x; }
	if (pt.y < min_y) { min_y = pt.y; }
	if (pt.y > max_y) { max_y = pt.y; }
      }

      // Need to expand bounds by 1 meter to avoid rounding problems
      // later.
      min_x -= 1.0;
      max_x += 1.0;
      min_y -= 1.0;
      max_y += 1.0;
      
      std::cerr << "Bounds for atlas: x "
		<< min_x << ", " << max_x
		<< "; y " << min_y << ", " << max_y
		<< std::endl;

      x_tile_size = (max_x - min_x) / num_x_tiles;
      y_tile_size = (max_y - min_y) / num_y_tiles; 

      std::cerr << "x_tile_size = " << x_tile_size << std::endl;
      std::cerr << "y_tile_size = " << y_tile_size << std::endl;

      // set tile sizes
      for (int i = 0; i < num_x_tiles; ++i) {
	for (int j = 0; j < num_y_tiles; ++j) {
	  tiles[i][j].set_min_bounds(min_x + i * x_tile_size, min_y + j * y_tile_size);
	  tiles[i][j].set_max_bounds(min_x + (i+1) * x_tile_size, min_y + (j+1) * y_tile_size);
	}
      }
      
      // add each point
      for (const auto& pt : cloud->points) {
	auto [x_idx, y_idx] = get_tile_indices(pt);
	tiles[x_idx][y_idx].add_point(pt);
      }
    }

    MapTile<PointT> get_tile(int i, int j) const {
      return tiles[i][j];
    }

    Landmark<PointT> extract_subcloud(const Point2& min_pt, const Point2& max_pt) {
      std::cerr << "min = " << min_pt << " and max = " << max_pt << std::endl;
      Landmark<PointT> landmark{min_pt, max_pt};
      
      // create new point cloud
      pcl::PointCloud<pcl::PointXYZI>::Ptr subcloud(new pcl::PointCloud<pcl::PointXYZI>);

      // iterate over main cloud
      for (const auto& pt : whole_cloud->points) {
	if (min_pt.x <= pt.x && pt.x <= max_pt.x &&
	    min_pt.y <= pt.y && pt.y <= max_pt.y) {
	  subcloud->push_back(pt);
	}
      }

      // write cloud file
      if (subcloud->points.size() > 0) {
	std::string filename = fmt::format("subcloud_{}.pcd", extraction_ctr);
	pcl::io::savePCDFileASCII(filename, *subcloud);
	
	// TODO write metadata file
	
	// increment counter
	extraction_ctr += 1;
      }

      landmarks.push_back(landmark);
      return landmark;
    }

    void serialize(YAML::Emitter& emitter) {
      emitter << YAML::BeginSeq;
      for (int i = 0; i < landmarks.size(); ++i) {
	emitter << YAML::BeginMap;
	emitter << YAML::Key << "index";
	emitter << YAML::Value << i;

	emitter << YAML::Key << "bounds";
	emitter << YAML::Value;
	landmarks[i].serialize(emitter);
	emitter << YAML::EndMap;
      }
      emitter << YAML::EndSeq;
    }
    
  };

} // namespace digital_twin


#endif // ATLAS_HPP
