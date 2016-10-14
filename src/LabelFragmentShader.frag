#version 330 core
out vec4 color;

uniform vec4 label_colour;

void main(){
  color = label_colour;
}
