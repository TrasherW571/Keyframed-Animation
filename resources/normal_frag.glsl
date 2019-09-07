#version 120
uniform vec3 lightPos;
uniform vec3 ka;
uniform vec3 kd;
uniform vec3 ks;
uniform float s;
uniform float i;
varying vec3 vNor;
varying vec3 vertexPos;

void main()
{
	vec3 color;
	vec3 n = normalize(vNor);
	vec3 lightVec = normalize(lightPos - vertexPos);
	vec3 Diff = kd * max(0.0, dot(n,lightVec));
	vec3 eyeVec = normalize(-vertexPos);
	vec3 halfVec = normalize(lightVec + eyeVec);
	vec3 Spec = ks * pow(max(0.0, dot(halfVec,n)), s);
	color = i * (ka + Diff + Spec);
	
	gl_FragColor = vec4(color, 1.0);
}
