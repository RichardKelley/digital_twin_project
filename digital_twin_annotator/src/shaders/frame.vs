#version 420 core

layout (location = 0) in vec3 pos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

flat out vec3 axis;

void main()
{
  axis = pos;
  gl_Position = projection * view * model * vec4(pos.x, pos.y, pos.z, 1.0);
}
 
