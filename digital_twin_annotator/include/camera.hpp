// Based on the Camera class from LearnOpenGL

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <iostream>

#include <GL/glew.h>
#include <GL/gl.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include <vector>

enum CameraMovement {
		     FORWARD,
		     BACKWARD,
		     LEFT,
		     RIGHT,
		     UP,
		     DOWN,
};

const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 50.0f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

enum CameraType {
		 FOLLOW,
		 OVERHEAD,
};

class Camera {
public:
  glm::vec3 position;
  glm::vec3 front;
  glm::vec3 up;
  glm::vec3 right;
  glm::vec3 world_up;

  float yaw;
  float pitch;

  float movement_speed;
  float mouse_sensitivity;
  float zoom;

  CameraType type;
  
  Camera(glm::vec3 p = glm::vec3(0.0f, 0.0f, 0.0f),
	 glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
	 glm::vec3 frnt = glm::vec3(0.0f, 0.0f, 1.0f),
	 float yaw = YAW, float pitch = PITCH, CameraType t = CameraType::FOLLOW)
    : front{frnt}
    , movement_speed{SPEED}
    , mouse_sensitivity{SENSITIVITY}
    , zoom{ZOOM}
    , type{t} {
      this->position = p;
      this->world_up = up;
      this->yaw = yaw;
      this->pitch = pitch;
      this->updateCameraVectors();
    }

  Camera(float posX, float posY, float posZ, float upX, float upY,
	 float upZ, float yaw, float pitch)
    : front{glm::vec3(0.0f, 1.0f, 0.0f)}
    , movement_speed{SPEED}
    , mouse_sensitivity{SENSITIVITY}
    , zoom{ZOOM} {
      this->position = glm::vec3(posX, posY, posZ);
      this->world_up = glm::vec3(upX, upY, upZ);
      this->yaw = yaw;
      this->pitch = pitch;
      this->updateCameraVectors();
    }

  glm::mat4 getViewMatrix() {
    return glm::lookAt(this->position,
		       this->position + this->front,
		       this->up);
  }

  void process_keyboard(CameraMovement direction, float dt) {
    float velocity = this->movement_speed * dt;
    if (direction == FORWARD) { this->position += this->front * velocity; }
    if (direction == BACKWARD) { this->position -= this->front * velocity; }
    if (direction == LEFT) { this->position -= this->right * velocity; }
    if (direction == RIGHT) { this->position += this->right * velocity; }
    if (direction == UP) { this->position += this->up * velocity; }
    if (direction == DOWN) { this->position -= this->up * velocity; }
    this->updateCameraVectors();
  }

  void process_mouse_movement(float xoffset, float yoffset, GLboolean constrain_pitch = true) {
    xoffset *= this->mouse_sensitivity;
    yoffset *= this->mouse_sensitivity;

    this->yaw += xoffset;
    this->pitch += yoffset;

    if (constrain_pitch) {
      if (this->pitch > 89.0f) {
	this->pitch = 89.0f;
      }

      if (this->pitch < -89.0f) {
	this->pitch = -89.0f;
      }
    }
    
    this->updateCameraVectors();
  }

  void process_mouse_scroll(float yoffset) {
    this->zoom -= (float)yoffset;
    if (this->zoom < 1.0f) {
      this->zoom = 1.0f;
    }

    if (this->zoom > 45.0f) {
      this->zoom = 45.0f;
    }
  }
  
private:

  void updateCameraVectors() {
    glm::vec3 front;
    front.x = cos(glm::radians(this->yaw)) * cos(glm::radians(this->pitch));
    front.y = sin(glm::radians(this->pitch));
    front.z = sin(glm::radians(this->yaw)) * cos(glm::radians(this->pitch));

    this->front = glm::normalize(front);    
    this->right = glm::normalize(glm::cross(this->front, this->world_up));
    this->up = glm::normalize(glm::cross(this->right, this->front));    
  }
  
};


#endif // CAMERA_HPP
