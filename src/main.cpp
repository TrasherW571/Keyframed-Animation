#include <iostream>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>


#include "Camera.h"
#include "GLSL.h"
#include "Program.h"
#include "MatrixStack.h"
#include "Shape.h"
#include "Helicopter.h"
#include "Light.h"
#include "Material.h"

using namespace std;
using namespace glm;

bool keyToggles[256] = {false}; // only for English keyboards!

GLFWwindow *window; // Main application window
string RESOURCE_DIR = ""; // Where the resources are loaded from

shared_ptr<Program> progNormal;
shared_ptr<Program> progSimple;
shared_ptr<Camera> camera;
shared_ptr<Shape> bunny;
vector<Helicopter> Components;
vector<vector<glm::vec3> > cps;	// vector of control points
vector<glm::quat> Q;
vector<vector<glm::quat> > Qcps;
vector<pair<float, float> > usTable;	// table that holds our u and s values
glm::mat4 Bcr;
glm::mat4 G;
glm::mat4 Rot;
glm::vec3 HelicopterPos(0,0,0);
glm::vec4 uVec;
bool NormCam = true;

static void error_callback(int error, const char *description)
{
	cerr << description << endl;
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GL_TRUE);
	}
}

static void char_callback(GLFWwindow *window, unsigned int key)
{
	keyToggles[key] = !keyToggles[key];
}

static void cursor_position_callback(GLFWwindow* window, double xmouse, double ymouse)
{
	if (!keyToggles[(unsigned)' '])
	{
		int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
		if (state == GLFW_PRESS) {
			camera->mouseMoved(xmouse, ymouse);
		}
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (!keyToggles[(unsigned)' '])
	{
		// Get the current mouse position.
		double xmouse, ymouse;
		glfwGetCursorPos(window, &xmouse, &ymouse);
		// Get current window size.
		int width, height;
		glfwGetWindowSize(window, &width, &height);
		if (action == GLFW_PRESS) {
			bool shift = mods & GLFW_MOD_SHIFT;
			bool ctrl = mods & GLFW_MOD_CONTROL;
			bool alt = mods & GLFW_MOD_ALT;
			camera->mouseClicked(xmouse, ymouse, shift, ctrl, alt);
		}
	}
}

// Builds the table that lists all the s values for the corresponding u points
void buildTable()
{
	glm::mat4 Gt;
	glm::vec4 Pa;
	glm::vec4 Pb;
	glm::vec4 P;
	int ncps = cps.size(); // number of control points
	float s = 0.0;
	float U = 0.0;

	usTable.push_back(make_pair(0.0f, 0.0f));
	for (int i = 0; i < ncps - 3; i++)
	{
		Gt[0] = glm::vec4(cps[i][0], 0.0f);
		Gt[1] = glm::vec4(cps[i][1], 0.0f);
		Gt[2] = glm::vec4(cps[i][2], 0.0f);
		Gt[3] = glm::vec4(cps[i][3], 0.0f);

		// need to keep little u between 0 and 1
		// calculates the arc length (s) for each u
		for (int j = 0; j < 5; j++)
		{
			float u = (float)j / 5;
			glm::vec4 uA(1.0f, u, u*u, u*u*u);
			U += 0.2;
			u += 0.2;
			glm::vec4 uB(1.0f, u, u*u, u*u*u);
			Pa = Gt * Bcr * uA;
			Pb = Gt * Bcr * uB;
			s += length(Pb - Pa);
			// Adds a new row to our table
			usTable.push_back(make_pair(U, s));
		}
	}
}

// finds the u value that corresponds to our given s value
float s2u(float s)
{
	bool inTable = false;
	int LowerIndex = 0;
	int UpperIndex = 1;
	float LowerB = usTable.at(LowerIndex).second;
	float UpperB = usTable.at(UpperIndex).second;
	float u;
	for (int i = 1; i < usTable.size(); i++)
	{
		if (usTable.at(i).second == s)
		{
			u = usTable.at(i).first;
			inTable = true;
			break;
		}
		else if (usTable.at(i).second > s)
		{
			break;
		}
		else
		{
			LowerB = usTable.at(i).second;
			LowerIndex = i;
			UpperB = usTable.at(i + 1).second;
			UpperIndex = i + 1;
		}
	}

	if (!inTable)
	{
		float alpha = (s - usTable.at(LowerIndex).second) / (usTable.at(UpperIndex).second - usTable.at(LowerIndex).second);
		u = ((1 - alpha) * usTable.at(LowerIndex).first) + (alpha * usTable.at(UpperIndex).first);
	}
	return u;
}

// does the dot product between two quaternions
bool DotProduct(const glm::quat q1, const glm::quat q2)
{
	float result = (q1.x * q2.x) + (q1.y * q2.y) + (q1.z * q2.z) + (q1.w * q2.w);
	if (result < 0)
	{
		return true;
	}
	return false;
}

// creates our rotation matrix to interpolate our helicopter along the spline curve
void RotationMatrix(double time)
{
	// max number of seconds it will take for helicopter to go through all keyframes
	float tMax = 5.0;
	// total length of the spline curve
	float sMax = usTable.at(usTable.size() - 1).second;
	//cout << "sMax: " << sMax << endl;
	float tNorm = std::fmod(time, tMax) / tMax;
	//cout << "Tnorm: " << tNorm << endl;
	float sNorm = tNorm;
	float s = sMax * sNorm;
	// Alpha is the linear interpolation parameter between 0 and 1, but right now its between 0 and tMax 
	float alpha = s2u(s);
	// transform range from (0, tMax) to (0,8)
	alpha = alpha * (8.0 / tMax);
	// figures out the control points for the segment we are in
	float cp = floor(alpha);
	// caluculates our alpha value
	alpha = alpha - cp;
	G[0] = glm::vec4(cps[cp][0], 0);
	G[1] = glm::vec4(cps[cp][1], 0);
	G[2] = glm::vec4(cps[cp][2], 0);
	G[3] = glm::vec4(cps[cp][3], 0);
	uVec = glm::vec4(1.0f, alpha, alpha*alpha, alpha*alpha*alpha);
	glm::vec4 pos = G * Bcr * uVec;

	// create the quaternions for each keyframe
	glm::vec3 Zaxis(0, 0, 1);
	glm::vec3 Xaxis(1, 0, 0);
	glm::vec3 Yaxis(0, 1, 0);
	glm::quat q0, q1, q2, q3, q4, q5, q6, q7;
	Xaxis = glm::normalize(Xaxis);
	Zaxis = glm::normalize(Zaxis);
	q0 = glm::angleAxis((float)0.0, Xaxis);
	q1 = glm::angleAxis((float)0.2, Zaxis);
	q1 *= glm::angleAxis((float)0.4, Xaxis);
	q2 = glm::angleAxis((float)0.7, Xaxis);
	q2 *= glm::angleAxis((float)0.2, Zaxis);
	q2 *= glm::angleAxis((float)3.14159, Yaxis);
	q3 = glm::angleAxis((float)0.3, Zaxis);
	q3 *= glm::angleAxis((float)-1.5708, Yaxis);
	q4 = glm::angleAxis((float)0.3, Zaxis);
	q4 *= glm::angleAxis((float)0.2, Xaxis);
	q5 = glm::angleAxis((float)0.2, Zaxis);
	q5 *= glm::angleAxis((float)1.5708, Yaxis);
	q6 = glm::angleAxis((float)0.3, Zaxis);
	q6 *= glm::angleAxis((float)0.3, Xaxis);
	q6 *= glm::angleAxis((float)3.92699, Yaxis);
	q7 = glm::angleAxis((float)0.3, Zaxis);

	Q.clear();
	Q.push_back(q0);
	Q.push_back(q0);
	Q.push_back(q1);
	Q.push_back(q2);
	Q.push_back(q3);
	Q.push_back(q4);
	Q.push_back(q5);
	Q.push_back(q6);
	Q.push_back(q7);
	Q.push_back(q0);
	Q.push_back(q0);

	// checks to see if the dot product between two successive quaternions is negative
	for (int i = 1; i < 9; i++)
	{
		// if the dot product is negative, then we negate q_(i+1) otherwise we see a weird twirl when the helicopter interpolates
		if (DotProduct(Q.at(i), Q.at(i + 1)))
		{
			Q.at(i + 1) = -Q.at(i + 1);
		}
	}
	// fills up our Qcps
	for (int i = 0; i < Q.size() - 3; i++)
	{
		vector<glm::quat> Temp;
		Temp.push_back(Q.at(i));
		Temp.push_back(Q.at(i + 1));
		Temp.push_back(Q.at(i + 2));
		Temp.push_back(Q.at(i + 3));
		Qcps.push_back(Temp);
	}
	glm::mat4 Gq;
	// each column of our Gq matrix contains the quaternion of the keyframe that it corresponds to
	Gq[0] = glm::vec4(Qcps[cp][0].x, Qcps[cp][0].y, Qcps[cp][0].z, Qcps[cp][0].w);
	Gq[1] = glm::vec4(Qcps[cp][1].x, Qcps[cp][1].y, Qcps[cp][1].z, Qcps[cp][1].w);
	Gq[2] = glm::vec4(Qcps[cp][2].x, Qcps[cp][2].y, Qcps[cp][2].z, Qcps[cp][2].w);
	Gq[3] = glm::vec4(Qcps[cp][3].x, Qcps[cp][3].y, Qcps[cp][3].z, Qcps[cp][3].w);
	glm::vec4 qVec = Gq * (Bcr * uVec);
	glm::quat q(qVec[3], qVec[0], qVec[1], qVec[2]); // (w, x, y, z)
	Rot = glm::mat4_cast(glm::normalize(q)); // Creates a rotation matrix
	Rot[3] = pos; // Puts 'pos', which is a vec3 that represents the position, into the last column
	Rot[3][3] = 1.0f;
}

void DrawKeyFrames(int index1, int index2, glm::mat4 R, shared_ptr<MatrixStack> &MV)
{
	MV->pushMatrix();
	MV->translate(cps[index1][index2]);
	MV->multMatrix(R);
	if (keyToggles[(unsigned)'k'])
	{
		Components.at(0).Draw(MV, progNormal);
		Components.at(1).Draw(MV, progNormal);
		Components.at(2).Draw(MV, progNormal);
		Components.at(3).Draw(MV, progNormal);
	}
	MV->popMatrix();
}
static void init()
{
	GLSL::checkVersion();
	
	// Set background color
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	// Enable z-buffer test
	glEnable(GL_DEPTH_TEST);
	
	keyToggles[(unsigned)'c'] = true;
	
	// For drawing the bunny
	progNormal = make_shared<Program>();
	progNormal->setShaderNames(RESOURCE_DIR + "normal_vert.glsl", RESOURCE_DIR + "normal_frag.glsl");
	progNormal->setVerbose(true);
	progNormal->init();
	progNormal->addUniform("P");
	progNormal->addUniform("MV");
	progNormal->addUniform("lightPos");
	progNormal->addUniform("CamPos");
	progNormal->addUniform("ka");
	progNormal->addUniform("kd");
	progNormal->addUniform("ks");
	progNormal->addUniform("s");
	progNormal->addUniform("i");
	progNormal->addAttribute("aPos");
	progNormal->addAttribute("aNor");
	progNormal->setVerbose(false);
	
	// For drawing the frames
	progSimple = make_shared<Program>();
	progSimple->setShaderNames(RESOURCE_DIR + "simple_vert.glsl", RESOURCE_DIR + "simple_frag.glsl");
	progSimple->setVerbose(true);
	progSimple->init();
	progSimple->addUniform("P");
	progSimple->addUniform("MV");
	progSimple->setVerbose(false);
	
	// Creates all the Helicopter Components
	Light L(0.0, 5.0, 1.0, 0.8);
	Material M(0.3, 0.3, 0.3, 1, 0, 0, 0, 0, 0, 200);			// body material (red)
	Material M2(0.3, 0.3, 0.3, 1, 1, 0, 0, 0, 0, 200);			// body material (yellow)
	Material M3(0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0, 0, 0, 200);	// Propellar material (gray)
	Helicopter H(0, 0, 0, 1, 0, 0, 0, "helicopter_body1.obj", RESOURCE_DIR, M, L);
	Helicopter H2(0, 0, 0, 1, 0, 0, 0, "helicopter_body2.obj", RESOURCE_DIR, M2, L);
	Helicopter H3(0, 0, 0, 0, 1, 0, 0, "helicopter_prop1.obj", RESOURCE_DIR, M3, L);
	Helicopter H4(0, 0, 0, 0, 0, 1, 0, "helicopter_prop2.obj", RESOURCE_DIR, M3, L);
	Components.push_back(H);
	Components.push_back(H2);
	Components.push_back(H3);
	Components.push_back(H4);
	
	
	camera = make_shared<Camera>();
	// Initializing keyframe positions
	glm::vec3 p0(0.0f, 0.0f, 0.0f);
	glm::vec3 p1(-3.5f, 0.8f, 0.0f);
	glm::vec3 p2(-1.7f, 3.5f, 3.5f);
	glm::vec3 p3(3.3f, 4.3f, 2.0f);
	glm::vec3 p4(2.5f, 5.0f, -3.0f);
	glm::vec3 p5(-1.9f, 2.8f, -1.0f);
	glm::vec3 p6(2.7f, 2.0f, 3.0f);
	glm::vec3 p7(2.2f, 0.8f, 0.0f);
	// Adding keyframe positions to control point vectors
	vector<glm::vec3> T;
	T.push_back(p0);
	T.push_back(p0);
	T.push_back(p1);
	T.push_back(p2);
	T.push_back(p3);
	T.push_back(p4);
	T.push_back(p5);
	T.push_back(p6);
	T.push_back(p7);
	T.push_back(p0);
	T.push_back(p0);
	for (int i = 0; i < T.size() - 3; i++)
	{
		vector<glm::vec3> Temp;
		Temp.push_back(T.at(i));
		Temp.push_back(T.at(i+1));
		Temp.push_back(T.at(i+2));
		Temp.push_back(T.at(i+3));
		cps.push_back(Temp);
	}

	// intialize our basis matrix
	Bcr[0] = vec4(0.0f, 2.0f, 0.0f, 0.0f);
	Bcr[1] = vec4(-1.0f, 0.0f, 1.0f, 0.0f);
	Bcr[2] = vec4(2.0f, -5.0f, 4.0f, -1.0f);
	Bcr[3] = vec4(-1.0f, 3.0f, -3.0f, 1.0f);
	Bcr = Bcr * 0.5f;
	// build the table for u and s values
	buildTable();
	// Initialize time.
	glfwSetTime(0.0);
	
	// If there were any OpenGL errors, this will print something.
	// You can intersperse this line in your code to find the exact location
	// of your OpenGL error.
	GLSL::checkError(GET_FILE_LINE);
}

void render()
{
	// Update time.
	double t = glfwGetTime();
	Components.at(2).setAngle(t*t);
	// center of position for the two propellars
	glm::vec3 TopCenter(0, 0.4819, 0);
	glm::vec3 SideCenter(0.6228, 0.1179, 0.1365);
	Components.at(2).setTranslations(TopCenter);
	Components.at(3).setAngle(t*t);
	Components.at(3).setTranslations(SideCenter);
	// Get current frame buffer size.
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
	
	// Use the window size for camera.
	glfwGetWindowSize(window, &width, &height);
	camera->setAspect((float)width/(float)height);
	
	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(keyToggles[(unsigned)'c']) {
		glEnable(GL_CULL_FACE);
	} else {
		glDisable(GL_CULL_FACE);
	}
	if(keyToggles[(unsigned)'l']) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	
	auto P = make_shared<MatrixStack>();
	auto MV = make_shared<MatrixStack>();
	
	// Apply camera transforms
	P->pushMatrix();
	camera->applyProjectionMatrix(P);
	MV->pushMatrix();
	// Creates the Quaternions for the Helicopter Frames
	RotationMatrix(t);
	// switches to helicopter camera mode
	if (keyToggles[(unsigned)' '])
	{
		// we take the Rot matrix we calculated and inverse it then add some offset and set it equal
		// to our View Matrix for our camera
		camera->setTranslations(glm::vec3(0,0,0));
		camera->setRotations(glm::vec2(0, 0));
		glm::mat4 Cam = Rot;
		Cam = glm::inverse(Cam);
		Cam[3][0] -= 5;
		MV->rotate(-1.5708, 0, 1, 0);
		MV->multMatrix(Cam);
		NormCam = true;
	}
	// switches to normal camera mode
	else
	{
		if (NormCam)
		{
			camera->setTranslations(glm::vec3(0, 0, -5));
			NormCam = false;
		}
		
	}
	camera->applyViewMatrix(MV);

	// Draw the 30x30 grid in XZ plane
	progSimple->bind();
	MV->pushMatrix();
	MV->translate(-15, -1, -15);
	glUniformMatrix4fv(progSimple->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
	glUniformMatrix4fv(progSimple->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	glLineWidth(2);
	// sets the color for inside each box
	glColor3f(1, 1, 1);
	glBegin(GL_QUADS);
		glVertex3f(0, -0.001, 0);
		glVertex3f(0, -0.001, 30);
		glVertex3f(30, -0.001, 30);
		glVertex3f(30, -0.001, 0);
	glEnd();
	glBegin(GL_LINES);
	for (int i = 0; i <= 30; i++)
	{
		if (i == 0) { glColor3f(.6, .3, .3); }
		else { glColor3f(.25, .25, .25); };
		glVertex3f(i, 0, 0);
		glVertex3f(i, 0, 30);
		if (i == 0) { glColor3f(.3, .3, .6); }
		else { glColor3f(.25, .25, .25); };
		glVertex3f(0, 0, i);
		glVertex3f(30, 0, i);
	}
	glEnd();
	glLineWidth(1);
	MV->popMatrix();
	MV->pushMatrix();
	glUniformMatrix4fv(progSimple->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
	glUniformMatrix4fv(progSimple->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));

	progSimple->bind();
	int ncps = cps.size();
	// iterates through all control points
	for (int i = 0; i < ncps; i++)
	{
		// Filling in geometry matrix column by column
		G[0] = vec4(cps[i][0], 0.0f);
		G[1] = vec4(cps[i][1], 0.0f);
		G[2] = vec4(cps[i][2], 0.0f);
		G[3] = vec4(cps[i][3], 0.0f);
		// Drawing Curve
		glBegin(GL_LINE_STRIP);
		// by changing the upper bound for u, the higher it is, the more smooth the curve will be
		for (int u = 0; u <= 100; ++u)
		{
			float U = u / 100.0f;
			// Fill in uVec
			uVec = glm::vec4(1.0f, U, U*U, U*U*U);
			// compute position at u
			vec4 p = G * (Bcr*uVec);
			if (keyToggles[(unsigned)'k'])
			{
				glColor3f(0, 0, 1);
				glVertex3f(p.x, p.y, p.z);
			}
		}
		glEnd();
	}
	MV->popMatrix();
	progSimple->unbind();
	GLSL::checkError(GET_FILE_LINE);
	
	progNormal->bind();
	// Send projection matrix (same for all bunnies)
	glUniformMatrix4fv(progNormal->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
	
	// The center of the helicopter is at (-0.2802, 0.932, 0.0851)
	glm::vec3 center(-0.2802, 0.932, 0.0851);
	MV->translate(0.2802, -0.932, -0.0851);
	MV->translate(0, 0.45, 0);
	
	// Rotation Matrices for each keyframe
	glm::mat4 R = glm::mat4_cast(Q.at(0));
	glm::mat4 R2 = glm::mat4_cast(Q.at(2));
	glm::mat4 R3 = glm::mat4_cast(Q.at(3));
	glm::mat4 R4 = glm::mat4_cast(Q.at(4));
	glm::mat4 R5 = glm::mat4_cast(Q.at(5));
	glm::mat4 R6 = glm::mat4_cast(Q.at(6));
	glm::mat4 R7 = glm::mat4_cast(Q.at(7));
	glm::mat4 R8 = glm::mat4_cast(Q.at(8));

	// Draws the Helicopter
	// Position 1 (starting/finishing position)
	DrawKeyFrames(0, 1, R, MV);
	// Position 2
	DrawKeyFrames(1, 1, R2, MV);
	// Position 3
	DrawKeyFrames(2, 1, R3, MV);
	// Position 4
	DrawKeyFrames(3, 1, R4, MV);
	// Position 5
	DrawKeyFrames(4, 1, R5, MV);
	// Position 6
	DrawKeyFrames(5, 1, R6, MV);
	// Position 7
	DrawKeyFrames(6, 1, R7, MV);
	// Position 8
	DrawKeyFrames(7, 1, R8, MV);

	// Interpolated Helicopter
	MV->pushMatrix();
		// calculates the spline interpolation
		MV->multMatrix(Rot);
		Components.at(0).Draw(MV, progNormal);
		Components.at(1).Draw(MV, progNormal);
		Components.at(2).Draw(MV, progNormal);
		Components.at(3).Draw(MV, progNormal);
	MV->popMatrix();
	progNormal->unbind();
	
	// Pop stacks
	MV->popMatrix();
	P->popMatrix();
	
	GLSL::checkError(GET_FILE_LINE);
}

int main(int argc, char **argv)
{
	if(argc < 2) {
		cout << "Please specify the resource directory." << endl;
		return 0;
	}
	RESOURCE_DIR = argv[1] + string("/");
	
	// Set error callback.
	glfwSetErrorCallback(error_callback);
	// Initialize the library.
	if(!glfwInit()) {
		return -1;
	}
	// Create a windowed mode window and its OpenGL context.
	window = glfwCreateWindow(640, 480, "YOUR NAME", NULL, NULL);
	if(!window) {
		glfwTerminate();
		return -1;
	}
	// Make the window's context current.
	glfwMakeContextCurrent(window);
	// Initialize GLEW.
	glewExperimental = true;
	if(glewInit() != GLEW_OK) {
		cerr << "Failed to initialize GLEW" << endl;
		return -1;
	}
	glGetError(); // A bug in glewInit() causes an error that we can safely ignore.
	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
	// Set vsync.
	glfwSwapInterval(1);
	// Set keyboard callback.
	glfwSetKeyCallback(window, key_callback);
	// Set char callback.
	glfwSetCharCallback(window, char_callback);
	// Set cursor position callback.
	glfwSetCursorPosCallback(window, cursor_position_callback);
	// Set mouse button callback.
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	// Initialize scene.
	init();
	// Loop until the user closes the window.
	while(!glfwWindowShouldClose(window)) {
		// Render scene.
		render();
		// Swap front and back buffers.
		glfwSwapBuffers(window);
		// Poll for and process events.
		glfwPollEvents();
	}
	// Quit program.
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
