#version 300 es

// Interpolated values from the vertex shaders
in mediump vec2 UV;

// Ouput data
out mediump vec3 color;

// Values that stay constant for the whole mesh.
uniform sampler2D myTextureSampler;

void main(){
    // Output color = color of the texture at the specified UV
    color = texture( myTextureSampler, UV ).rgb;
}
