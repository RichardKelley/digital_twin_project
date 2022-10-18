#include <map_tile_mesh.hpp>
#include <shader.hpp>

#include <GL/glew.h>
#include <GL/gl.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace digital_twin {

  void MapTileMesh::draw(const glm::mat4& proj, const glm::mat4& view) {
    tile_shader.use();

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, this->position);
    model = model * orientation.rotation_matrix();
    
    tile_shader.setMat4("projection", proj);
    tile_shader.setMat4("view", view);
    tile_shader.setMat4("model", model);
    
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, vbo_data.size() / 4);
    glBindVertexArray(0);    
  }

  void MapTileMesh::setupMesh() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vbo_data.size(), &vbo_data[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

} // namespace digital_twin
