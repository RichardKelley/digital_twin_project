#ifndef MAP_TILE_MESH_HPP
#define MAP_TILE_MESH_HPP

#include <iostream>
#include <vector>
#include <algorithm>

#include <GL/glew.h>
#include <GL/gl.h>

#include "shader.hpp"
#include "drawable.hpp"
#include "quaternion.hpp"

#include <glm/glm.hpp>

namespace digital_twin {
  
  struct MapTileMesh : public Drawable {

    MapTileMesh(const std::vector<float>& points)
      : tile_shader{"../src/shaders/pointcloud.vs", "../src/shaders/pointcloud.fs"}
      , vbo_data(points.size())
      , position{glm::vec3(0.0, 0.0, 0.0)}
      , orientation{} {
      std::copy(points.begin(), points.end(), vbo_data.begin());
      setupMesh();
    }

    ~MapTileMesh() {
      tile_shader.cleanup();
      glDeleteBuffers(1, &vbo);
      glDeleteVertexArrays(1, &vao);
    }
    
    void draw(const glm::mat4& proj, const glm::mat4& view) override;
    void setupMesh();

    std::vector<float> vbo_data;
    
    glm::vec3 position;
    quaternion<float> orientation;

    GLuint vao;
    GLuint vbo;
    Shader tile_shader;    
  };  

} // namespace digital_twin

#endif // MAP_TILE_MESH_HPP
