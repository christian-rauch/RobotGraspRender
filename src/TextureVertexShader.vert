#version 300 es

// Input vertex data, different for all executions of this shader.
layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 4) in vec2 vertexUV;
//layout(location = 3) in vec2 vertexUV;
//layout(location = 0) in vec2 vertexUV;

// Output data ; will be interpolated for each fragment.
out vec2 UV;

// Values that stay constant for the whole mesh.
uniform mat4 MVP;
uniform mat4 M; // ModelMatrix

void main(){

    // Output position of the vertex, in clip space : MVP * position
    gl_Position =  MVP * M * vec4(vertexPosition_modelspace,1);

    // UV of the vertex. No special space for this one.
    UV = vertexUV;
}
