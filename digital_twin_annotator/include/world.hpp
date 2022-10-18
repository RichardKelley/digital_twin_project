#ifndef WORLD_HPP
#define WORLD_HPP

#include <vector>
#include <memory>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "atlas.hpp"
#include "map_tile.hpp"
#include "landmark.hpp"
#include "drawable.hpp"

namespace digital_twin {

  // everything we want to draw.
  class World {
  public:
    void draw(const glm::mat4& proj, const glm::mat4& view) {
      for (const auto& o : scene) {
	o->draw(proj, view);
      }
    }

    void add(const std::shared_ptr<Drawable>& obj) {
      scene.push_back(obj);
    }

  private:
    
    std::vector<std::shared_ptr<Drawable>> scene;
    
  };

} // namespace digital_twin


#endif // WORLD_HPP
