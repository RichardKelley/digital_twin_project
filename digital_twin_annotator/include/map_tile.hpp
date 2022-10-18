#ifndef MAP_TILE_HPP
#define MAP_TILE_HPP

#include <vector>

#include <fmt/core.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace digital_twin {

  /*
     A map tile is a subset of a larget point cloud, defined by a
     rectangle whose corners are specified in the map frame.

     Invariants:

     - The tile_cloud should contain only points whose (x,y) lie in
       the rectangle specified in the constructor.

       @author Richard Kelley
  */
  template<typename PointT>
  class MapTile {
    float min_x = 0.0f;
    float min_y = 0.0f;
    float max_x = 0.0f;
    float max_y = 0.0f;

    // not user-defined. Used to construct bounding volumes for
    // visualization.
    float min_z = 0.0f;
    float max_z = 0.0f;
    
    pcl::PointCloud<PointT>::Ptr tile_cloud;
    
  public:

    MapTile() : tile_cloud{new pcl::PointCloud<PointT>} { }
    
    MapTile(float min_x, float min_y, float max_x, float max_y)
      : min_x{min_x}, min_y{min_y}, max_x{max_x}, max_y{max_y}
      , tile_cloud{new pcl::PointCloud<PointT>} { }

    ~MapTile() = default;

    float get_min_x() const { return min_x; }
    float get_min_y() const { return min_y; }
    float get_max_x() const { return max_x; }
    float get_max_y() const { return max_y; }

    float get_min_z() const { return min_z; }
    float get_max_z() const { return max_z; }

    void set_min_bounds(float new_min_x, float new_min_y) {
      min_x = new_min_x;
      min_y = new_min_y;
    }

    void set_max_bounds(float new_max_x, float new_max_y) {
      max_x = new_max_x;
      max_y = new_max_y;
    }    
    
    std::string to_string() const {
      return fmt::format("MapTile(min_x = {}, max_x = {}, min_y = {}, max_y = {}, size = {})",
			 min_x, max_x, min_y, max_y, tile_cloud->size());	    
    }
    
    /*
      Return true iff pt has coordinates that lie in this tile.
    */
    bool contains(const PointT& pt) {
      return (min_x <= pt.x &&
	      pt.x <= max_x &&
	      min_y <= pt.y &&
	      pt.y <= max_y);
    }

    /*
      Add a point to this tile iff the point is in the correct (x,y)
      intervals.
    */
    void add_point(const PointT& pt) {
      if (this->contains(pt)) {
	tile_cloud->push_back(pt);

	if (pt.z > max_z) max_z = pt.z;
	if (pt.z < min_z) min_z = pt.z;
      }
    }    

    pcl::PointCloud<PointT> get_cloud() const {
      return *tile_cloud;
    }
    
    /*
      Return a vector of floats in a format that can be passed to a
      VBO for OpenGL rendering.
    */
    std::vector<float> to_vbo_data() const {
      std::vector<float> vbo_data;
      for (const auto& pt : tile_cloud->points) {
	vbo_data.push_back(pt.x);
	vbo_data.push_back(pt.y);
	vbo_data.push_back(pt.z);
	vbo_data.push_back(pt.intensity);
      }
      return vbo_data;
    }

  }; 

} // namespace digital_twin

#endif // MAP_TILE_HPP
