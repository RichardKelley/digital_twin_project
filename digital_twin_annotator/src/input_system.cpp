#include <input_system.hpp>

#include <SDL2/SDL.h>

namespace digital_twin {

  void InputSystem::init(RenderSystem& render_system) {
    last_x = render_system.get_width() / 2.0;
    last_y = render_system.get_height() / 2.0;
  }

  // return true iff program is done.
  bool InputSystem::update(RenderSystem& render_system, float dt) {
    SDL_Event e;

    int mouse_x_int;
    int mouse_y_int;

    float mouse_x, mouse_y;
    float xoffset, yoffset;

    SDL_PumpEvents();
    this->key_state = SDL_GetKeyboardState(nullptr);
    
    if (key_state[SDL_SCANCODE_A]) {
      render_system.camera.process_keyboard(LEFT, dt);
    }
    
    if (key_state[SDL_SCANCODE_S]) {
      render_system.camera.process_keyboard(BACKWARD, dt);
    }
    
    if (key_state[SDL_SCANCODE_D]) {
      render_system.camera.process_keyboard(RIGHT, dt);
    }
    
    if (key_state[SDL_SCANCODE_W]) {
      render_system.camera.process_keyboard(FORWARD, dt);
    }
    
    if (key_state[SDL_SCANCODE_E]) {
      render_system.camera.process_keyboard(UP, dt);	
    }
    
    if (key_state[SDL_SCANCODE_Q]) {
      render_system.camera.process_keyboard(DOWN, dt);
    }
    
    /*
      if (key_state[SDL_SCANCODE_TAB]) {
      std::cerr << "TAB" << std::endl;
      }
    */
    
    if (key_state[SDL_SCANCODE_ESCAPE]) {
      return true;
    }
    
    while (SDL_PollEvent(&e)) {
      switch (e.type) {
      case SDL_QUIT:
	return true;
	break;
	
      case SDL_KEYDOWN:
	/*
	switch(e.key.keysym.sym) {	  
	case SDLK_TAB:
	  break;
	}
	*/
	break;
	
      case SDL_MOUSEBUTTONDOWN:
	switch(e.button.button) {
	case SDL_BUTTON_LEFT:
	  SDL_GetMouseState(&mouse_x_int, &mouse_y_int);
	  mouse_x = static_cast<float>(mouse_x_int);
	  mouse_y = static_cast<float>(mouse_y_int);
	  render_system.process_click(mouse_x, mouse_y);
	  break;
	    
	case SDL_BUTTON_MIDDLE:
	  this->middle_clicked = true;
	  break;
	default:
	  break;
	}
	break;
	
      case SDL_MOUSEBUTTONUP:
	switch(e.button.button) {
	case SDL_BUTTON_MIDDLE:
	  this->middle_clicked = false;
	  last_x = render_system.get_width() / 2.0;
	  last_y = render_system.get_height() / 2.0;
	  first_mouse = true;
	  break;
	default:
	  break;
	}
	break;
	
      case SDL_MOUSEMOTION:
	if (this->middle_clicked && render_system.camera.type == CameraType::FOLLOW) {
	  SDL_GetMouseState(&mouse_x_int, &mouse_y_int);
	  
	  mouse_x = static_cast<float>(mouse_x_int);
	  mouse_y = static_cast<float>(mouse_y_int);
	  
	  if (first_mouse) {
	    last_x = mouse_x;
	    last_y = mouse_y;
	    first_mouse = false;
	  }

	  xoffset = mouse_x - last_x;
	  yoffset = mouse_y - last_y;
	  
	  last_x = mouse_x;
	  last_y = mouse_y;
	  
	  render_system.camera.process_mouse_movement(xoffset, yoffset);
	}
	break;
	
      default:
	break;	
      }
    }
    
    return false;
    
  }

} // namespace digital_twin
