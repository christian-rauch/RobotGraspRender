#version 300 es
layout(location = 0) in vec3 vertexPosition_modelspace;
//layout(location = 1) in vec3 vertexColor; // red
//layout(location = 2) in vec3 vertexColor; // blue
layout(location = 3) in vec3 vertexColor;   // this is our colour

out vec3 fragmentColor;

uniform mat4 MVP;
uniform mat4 M; // ModelMatrix

void main(){
    gl_Position =  MVP * M * vec4(vertexPosition_modelspace,1);
    fragmentColor = vertexColor;
}
