#include "shader.hpp"
#include "render_system.hpp"
#include "world.hpp"

namespace digital_twin {
  
  bool RenderSystem::init() {
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
      std::cerr << "SDL_Init error." << std::endl;
      return false;
    }
    
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    
    main_window = SDL_CreateWindow("Digital Twin Annotator", SDL_WINDOWPOS_CENTERED,
				   SDL_WINDOWPOS_CENTERED, 1024, 768,
				   SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    
    if (main_window == nullptr) {
      std::cerr << SDL_GetError() << std::endl;
      return false;
    }
    SDL_SetWindowResizable(main_window, SDL_FALSE);
    
    gl_context = SDL_GL_CreateContext(main_window);
    if (gl_context == nullptr) {
      std::cerr << SDL_GetError() << std::endl;
      return false;
    }
    
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err) {
      std::cerr << "GLEW initialization error." << std::endl;
      return false;
    }
    
    screen_width = 1024;
    screen_height = 768;
    
    SDL_GL_SetSwapInterval(0);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glViewport(0.0, 0.0, 1024.0, 768.0);
    
    // TODO initialize text rendering.
    
    // TODO initialize main framebuffer
    screen_shader = Shader{"../src/shaders/screen.vs", "../src/shaders/screen.fs"};

    float screen_vertices[] = {
			       // positions   // texCoords
			       -1.0f,  1.0f,  0.0f, 1.0f,
			       -1.0f, -1.0f,  0.0f, 0.0f,
			       1.0f, -1.0f,  1.0f, 0.0f,    
			       -1.0f,  1.0f,  0.0f, 1.0f,
			       1.0f, -1.0f,  1.0f, 0.0f,
			       1.0f,  1.0f,  1.0f, 1.0f
			       
    };

    GLuint screen_vbo;
    glGenVertexArrays(1, &screen_vao);
    glGenBuffers(1, &screen_vbo);

    glBindVertexArray(screen_vao);
    glBindBuffer(GL_ARRAY_BUFFER, screen_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(screen_vertices), &screen_vertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2*sizeof(float)));
    glBindVertexArray(0);

    screen_shader.use();
    screen_shader.setInt("screenTexture", 0);

    // set up screen FBO
    glGenFramebuffers(1, &screen_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, screen_fbo);

    glGenTextures(1, &color_texture_attachment);
    glBindTexture(GL_TEXTURE_2D, color_texture_attachment);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
		 this->screen_width, this->screen_height,
		 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
			   color_texture_attachment, 0);

    // set up screen RBO
    glGenRenderbuffers(1, &screen_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, screen_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, screen_width, screen_height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
			      GL_RENDERBUFFER, screen_rbo);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      std::cerr << "Framebuffer is not complete!" << std::endl;
      return false;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
    return true;  
  }


  void RenderSystem::resize() {
    // TODO    
  }

  //  void RenderSystem::render(World& w) {
  void RenderSystem::render() {
    glm::mat4 projection;

    float aspect = screen_width / screen_height;
    float cam_dist = camera.position.z;
    projection = glm::ortho(-cam_dist * aspect, cam_dist * aspect, -cam_dist, cam_dist, -200.0f, 200.0f);

    glm::mat4 view = camera.getViewMatrix();
        
    glBindFramebuffer(GL_FRAMEBUFFER, screen_fbo);
    glClearColor(0.1, 0.1, 0.1, 1.0);
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    w->draw(projection, view);
    
    // draw screen
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_DEPTH_TEST);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    screen_shader.use();
    glBindVertexArray(this->screen_vao);
    glBindTexture(GL_TEXTURE_2D, color_texture_attachment);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    // TODO I/O pass
    
    SDL_GL_SwapWindow(main_window);

  }

  void RenderSystem::process_click(int x, int y) {

    
    // flip y
    float x_fl = static_cast<float>(x);
    float y_fl = static_cast<float>(screen_height - y);

    // invert viewport
    float x_d = (2.0 / screen_width) * x_fl - 1;
    float y_d = (2.0 / screen_height) * y_fl - 1;
    
    // invert ortho
    float aspect = screen_width / screen_height;
    float cam_dist = camera.position.z;
    glm::mat4 proj = glm::ortho(-cam_dist * aspect, cam_dist * aspect, -cam_dist, cam_dist, -200.0f, 200.0f);
    glm::mat4 inv_proj = glm::inverse(proj);

    glm::vec4 v_d(x_d, y_d, 0.0, 1.0);

    glm::vec4 v_e = inv_proj * v_d;

    float x_e = v_e.x;
    float y_e = v_e.y;

    // invert view
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 view_inv = glm::inverse(view);

    glm::vec4 v_w = view_inv * v_e;

    float x_w = v_w.x;
    float y_w = v_w.y;

    if (min_now == 1) {
      min_pt = Point2{.x = x_w, .y = y_w};
    } else {
      max_pt = Point2{.x = x_w, .y = y_w};
      Landmark<pcl::PointXYZI> lm = atlas->extract_subcloud(min_pt, max_pt);
      landmarks.push_back(lm);
      w->add(std::make_shared<Landmark<pcl::PointXYZI>>(lm));
    }
    min_now = 1 - min_now;
    
  }

  
  void RenderSystem::shutdown() {
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(main_window);
    SDL_Quit();
  }
  


} // namespace digital_twin
