#pragma once
#include <glm/glm.hpp>
#include <GL/glew.h>
using namespace std;

class Material
{
public:
Material::Material()
{
	Ambient = glm::vec3(0.3, 0.3, 0.3);
	Diffuse = glm::vec3(1, 0, 0);
	Specular = glm::vec3(0, 0, 0);
	Shininess = 200;
}
Material::Material(float Ax, float Ay, float Az, float Dx, float Dy, float Dz, float Sx, float Sy, float Sz, float shiny)
	:Ambient(Ax,Ay,Az), Diffuse(Dx,Dy,Dz), Specular(Sx,Sy,Sz)
{
	Shininess = shiny;
}
~Material()
{
}
glm::vec3 getAmbient() { return Ambient; }
glm::vec3 getDiffuse() { return Diffuse; }
glm::vec3 getSpecular() { return Specular; }
float getShiny() { return Shininess; }
private:
glm::vec3 Ambient;
glm::vec3 Diffuse;
glm::vec3 Specular;
float Shininess;
};