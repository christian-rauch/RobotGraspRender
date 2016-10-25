#version 300 es
in mediump vec3 fragmentColor;

out mediump vec3 color;

void main(){
  color = fragmentColor;
//  color = vec3(1,1,0);
}
