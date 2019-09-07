#pragma once
#include <glm/glm.hpp>
#include <GL/glew.h>
#include <memory>
#include "Shape.h"
#include "Material.h"
#include "Light.h"
using namespace std;

class Helicopter
{
public:
	Helicopter(float tx, float ty, float tz, float rx, float ry, float rz, float A, string file, string Resource, Material m, Light l)
	: translations(tx, ty, tz), rotations(rx, ry, rz), angle(A)
	{
		M = m;
		L = l;
		shape = make_shared<Shape>();
		shape->loadMesh(Resource + file);
		shape->init();
	}
	~Helicopter() {}
	glm::vec3 getTranslations()
	{
		return translations;
	}
	glm::mat4 getCurrentMtrix()
	{
		glm::mat4 Current(1.0);
		Current *= glm::translate(glm::mat4(1.0f), translations);
		Current *= glm::rotate(glm::mat4(1.0f), angle, rotations);
		Current *= glm::translate(glm::mat4(1.0f), -translations);
		return Current;
	}
	// Draws the object
	void Draw(shared_ptr<MatrixStack> &MV, shared_ptr<Program> &prog) 
	{
		MV->pushMatrix();
		MV->translate(translations);
		MV->rotate(angle, rotations);
		MV->translate(-translations);
		glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
		glUniform3f(prog->getUniform("lightPos"), L.getLightPosition().x, L.getLightPosition().y, L.getLightPosition().z);
		glUniform3f(prog->getUniform("ka"), M.getAmbient().x, M.getAmbient().y, M.getAmbient().z);
		glUniform3f(prog->getUniform("kd"), M.getDiffuse().x, M.getDiffuse().y, M.getDiffuse().z);
		glUniform3f(prog->getUniform("ks"), M.getSpecular().x, M.getSpecular().y, M.getSpecular().z);
		glUniform1f(prog->getUniform("s"), M.getShiny());
		glUniform1f(prog->getUniform("i"), L.getIntensity());
		shape->draw(prog);
		MV->popMatrix();
	}
	// Setter Functions
	void setTranslations(glm::vec3 move)
	{
		translations.x = move.x;
		translations.y = move.y;
		translations.z = move.z;
	}
	void setRotations(float rx, float ry, float rz)
	{
		rotations.x = rx;
		rotations.y = ry;
		rotations.z = rz;
	}
	void setAngle(float A) { angle = A; }
private:
	glm::vec3 translations;
	glm::vec3 rotations;
	glm::vec3 Diffuse;
	Material M;
	Light L;
	shared_ptr<Shape> shape;
	float angle;
};