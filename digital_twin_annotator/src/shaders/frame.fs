#version 420 core

flat in vec3 axis;
out vec4 FragColor;

void main() {
  FragColor = vec4(axis, 1.0);
}
