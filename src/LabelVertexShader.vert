#version 330 core
layout(location = 0) in vec3 vertexPosition_modelspace;

uniform mat4 MVP;
uniform mat4 M; // ModelMatrix

void main(){
    gl_Position =  MVP * M * vec4(vertexPosition_modelspace,1);
}
