#version 120
attribute vec4 aPos;
attribute vec3 aNor;
uniform mat4 P;
uniform mat4 MV;
varying vec3 vNor;
varying vec3 vertexPos;

void main()
{
	gl_Position = P * MV * aPos;
	vertexPos = vec3(MV * aPos);
	vNor = (MV * vec4(aNor, 0.0)).xyz;
}
