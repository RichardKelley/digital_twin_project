#version 420 core

in float intensity;

out vec4 FragColor;

void main() {
  FragColor = vec4(intensity, intensity, intensity, 1.0);
}
