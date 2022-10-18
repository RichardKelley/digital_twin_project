#ifndef LANDMARK_HPP
#define LANDMARK_HPP

#include <string>

#include <point2.hpp>
#include <drawable.hpp>
#include <quaternion.hpp>
#include <shader.hpp>

#include <glm/glm.hpp>

#include <yaml-cpp/yaml.h>

namespace digital_twin {

  /*
    A Landmark is defined as a rectangle containing an object of interest.
  */
  template<typename PointT>
  struct Landmark : public Drawable {

    Landmark(const Point2& min, const Point2& max)
      : min_x{min.x}
      , min_y{min.y}
      , max_x{max.x}
      , max_y{max.y}
      , shader {"../src/shaders/landmark.vs", "../src/shaders/landmark.fs"} {
      setupMesh();
    }

    Landmark(const Landmark& other)
      : min_x{other.min_x}
      , min_y{other.min_y}
      , max_x{other.max_x}
      , max_y{other.max_y}
      , shader{"../src/shaders/landmark.vs", "../src/shaders/landmark.fs"} {
      setupMesh();
    }
    
    std::string name{""};    
    float min_x = 0.0f;
    float min_y = 0.0f;
    float max_x = 0.0f;
    float max_y = 0.0f;

    // OpenGL data
    glm::vec3 position;
    quaternion<float> orientation;

    GLuint vao;
    GLuint vbo;
    Shader shader;
    
    void draw(const glm::mat4& proj, const glm::mat4& view) override {
      shader.use();

      glm::mat4 model = glm::mat4(1.0f);
      model = glm::translate(model, this->position);
      model = model * orientation.rotation_matrix();
    
      shader.setMat4("projection", proj);
      shader.setMat4("view", view);
      shader.setMat4("model", model);
    
      glBindVertexArray(vao);
      glDrawArrays(GL_TRIANGLES, 0, 6);
      glBindVertexArray(0);    

    }
    
    void setupMesh() {      
      glGenVertexArrays(1, &vao);
      glGenBuffers(1, &vbo);

      glBindVertexArray(vao);
      glBindBuffer(GL_ARRAY_BUFFER, vbo);

      float depth = -100.0f;
      float vertices[] = {
	min_x, max_y, depth,
	min_x, min_y, depth,
	max_x, min_y, depth,

	min_x, max_y, depth,
	max_x, min_y, depth,
	max_x, max_y, depth
      };
      
      glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices, GL_STATIC_DRAW);
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

      glBindVertexArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void serialize(YAML::Emitter& emitter) {
      emitter << YAML::BeginMap;
      emitter << YAML::Key << "min_x";
      emitter << YAML::Value << min_x;

      emitter << YAML::Key << "min_y";
      emitter << YAML::Value << min_y;

      emitter << YAML::Key << "max_x";
      emitter << YAML::Value << max_x;

      emitter << YAML::Key << "max_y";
      emitter << YAML::Value << max_y;
      
      emitter << YAML::EndMap;	
    }

    
  };


} // namespace digital_twin

#endif // LANDMARK_HPP
