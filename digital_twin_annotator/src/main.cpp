#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <memory>
#include <fstream>

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "atlas.hpp"
#include "map_tile_mesh.hpp"
#include "input_system.hpp"
#include "render_system.hpp"

#include <GL/glew.h>
#include <GL/gl.h>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Need to specify a map to load." << std::endl;
    return 1;
  }

  Camera overhead{glm::vec3(0.0f, 0.0f, 30.0f),
		  glm::vec3(0.0f, 1.0f, 0.0f),
		  glm::vec3(0.0f, 0.0f, 1.0f)};
  overhead.type = CameraType::OVERHEAD;    
  
  digital_twin::InputSystem input_system;
  digital_twin::RenderSystem render_system{overhead};
  if (!render_system.init()) {
    std::cerr << "RenderSystem initialization failed." << std::endl;
    return 1;
  }
  input_system.init(render_system);
  
  std::string pcd_filename{argv[1]};
  std::cerr << "Attempting to open " << pcd_filename << "..." << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename, *map_cloud) == -1) {
    std::cerr << "Failed to load map." << std::endl;
    return 2;
  }

  std::cerr << "Loaded a cloud with "
	    << map_cloud->width * map_cloud->height
	    << " points." << std::endl;

  std::cerr << "Creating an atlas with 10x10 tiles" << std::endl;

  digital_twin::Atlas<pcl::PointXYZI> atlas{map_cloud, 10, 10};
  digital_twin::World world;
  render_system.set_atlas(&atlas);
  render_system.set_world(&world);
  
  std::cerr << "adding map tile meshes." << std::endl;
  
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 10; ++j) {
      auto tile = atlas.get_tile(i,j);
      world.add(std::make_shared<digital_twin::MapTileMesh>(tile.to_vbo_data()));
    }
  }
  
  bool done = false;
  float dt = 0.0;

  using namespace std::literals::chrono_literals;

  YAML::Emitter emitter;
  
  std::cerr << "Current directory: " << fs::current_path() << std::endl;
  if (fs::exists("atlas")) {
    fs::remove_all("atlas");
  }
  
  std::cerr << "Starting main loop..." << std::endl;
  while (!done) {
    auto start_time = std::chrono::system_clock::now();
    
    // process inputs
    done = input_system.update(render_system, dt);
    
    render_system.render();
    
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    dt = elapsed_seconds.count();
    
    if (elapsed_seconds < 8ms) {
      std::this_thread::sleep_for(8ms - elapsed_seconds);
    }
  }
  

  render_system.shutdown();

  atlas.serialize(emitter);
  std::ofstream yaml_out{"landmarks.yaml"};

  yaml_out << emitter.c_str() << std::endl;

  fs::create_directory("atlas");
  fs::copy("./landmarks.yaml", "atlas/");
  fs::remove("./landmarks.yaml");

  for (int i = 0; i < atlas.get_num_landmarks(); ++i) {
    fs::copy(fmt::format("subcloud_{}.pcd", i), "atlas/");
    fs::remove(fmt::format("subcloud_{}.pcd", i));
  }
}
