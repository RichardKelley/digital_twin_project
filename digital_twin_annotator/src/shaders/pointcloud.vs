#version 420 core

layout (location = 0) in vec4 pos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out float intensity;

void main() {
  intensity = pos.w / 64.0;
  gl_Position = projection * view * model * vec4(pos.x, pos.y, pos.z, 1.0);
}
