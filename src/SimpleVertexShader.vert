#version 330 core
layout(location = 0) in vec3 vertexPosition_modelspace;
//layout(location = 1) in vec3 vertexColor; // red
//layout(location = 2) in vec3 vertexColor; // blue
layout(location = 3) in vec3 vertexColor;   // this is our colour

out vec3 fragmentColor;

uniform mat4 MVP;

void main(){
//    gl_Position.xyz = vertexPosition_modelspace;
//    gl_Position.w = 1.0;

    gl_Position =  MVP * vec4(vertexPosition_modelspace,1);
    fragmentColor = vertexColor;
}
