#ifndef RENDER_SYSTEM_HPP
#define RENDER_SYSTEM_HPP

#include <world.hpp>
#include <camera.hpp>
#include <shader.hpp>
#include <atlas.hpp>
#include <point2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include <map>
#include <string>

#include <glm/glm.hpp>

#include <GL/glew.h>
#include <GL/gl.h>

#include <SDL2/SDL.h>

namespace digital_twin {

  class RenderSystem {
  public:

    RenderSystem() = default;

    RenderSystem(Camera cam) : camera{cam} { }

    bool init();
    void resize();

    //void render(World& w);
    void render();
    
    void shutdown();

    inline float get_width() const {
      return screen_width;
    }

    inline float get_height() const {
      return screen_height;
    }

    void process_click(int x, int y);
    
    Camera camera;    

    inline void set_atlas(Atlas<pcl::PointXYZI>* atlas) {
      this->atlas = atlas;
    }

    inline void set_world(World* world) {
      w = world;
    }
    
  private:

    SDL_Window *main_window;
    SDL_GLContext gl_context;
    
    World* w;

    float screen_width;
    float screen_height;

    // framebuffer variables
    GLuint screen_fbo;
    GLuint color_texture_attachment;
    GLuint screen_rbo;
    GLuint depth_stencil_attachment;
    
    // screen variables
    Shader screen_shader;
    GLuint screen_vao;

    Atlas<pcl::PointXYZI>* atlas;

    std::vector<Landmark<pcl::PointXYZI>> landmarks;
    
    Point2 min_pt;
    Point2 max_pt;
    int min_now = 1;
  };

}

#endif // RENDER_SYSTEM_HPP
