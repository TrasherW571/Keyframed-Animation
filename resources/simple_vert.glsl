#version 120

uniform mat4 P;
uniform mat4 MV;
varying vec3 fragColor;

void main()
{
	gl_Position = P * MV * gl_Vertex;
	fragColor = gl_Color.rgb;
}
