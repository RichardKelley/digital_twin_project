#ifndef INPUT_SYSTEM_HPP
#define INPUT_SYSTEM_HPP

#include "render_system.hpp"

#include <SDL2/SDL.h>

#include <iostream>

namespace digital_twin {

  class InputSystem {

  public:
    void init(RenderSystem& render_system);
    bool update(RenderSystem& render_system, float dt);

    const Uint8* key_state;
    bool middle_clicked = false;

    float last_x;
    float last_y;

    bool first_mouse = true;    
  };

} // namespace digital_twin


#endif // INPUT_SYSTEM_HPP
