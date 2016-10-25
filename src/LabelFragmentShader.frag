#version 300 es
out mediump vec4 color;

uniform mediump vec4 label_colour;

void main(){
  color = label_colour;
}
