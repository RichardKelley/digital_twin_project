#ifndef DRAWABLE_HPP
#define DRAWABLE_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace digital_twin {

  /*
   * Abstract base class for a drawable object in the simulation world.
   */
  struct Drawable {
    virtual void draw(const glm::mat4& proj, const glm::mat4& view) = 0;
  };

}

#endif // DRAWABLE_HPP
