/*
 *	Includes
 */
#include <algorithm>
#include <assimp/cimport.h>		// C importer
#include <assimp/scene.h>		// Collects data
#include <assimp/postprocess.h> // Various extra operations
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iomanip>				// setprecision
#include <iostream>
#include <math.h>
#include <mmsystem.h>
#include <queue>
#include <sstream>
#include <stdio.h>
#include <vector>				// STL dynamic memory
#include <windows.h>

#include "Antons_maths_funcs.h" // Anton's maths functions
#include "Camera.h"
#include "Distance.h"
#include "Mesh.h"
#include "Model.h"
#include "PlaneRotation.h"
#include "RigidBody.h"
#include "Shader_Functions.h"
#include "text.h"
#include "time.h"

using namespace std;

/*
 *	Globally defined variables and constants
 */
//#define BUFFER_OFFSET(i) ((char *)NULL + (i))  // Macro for indexing vertex buffer

#define NUM_MESHES   3
#define NUM_SHADERS	 5
#define NUM_TEXTURES 2

bool firstMouse = true;
bool keys[1024];
bool pause = false;
Camera camera(vec3(0.0f, 0.0f, 4.0f));
enum Meshes { PLANE_MESH, ASTEROID_MESH, SPHERE_MESH };
enum Mode { BOUNDING_SPHERES, AABB };
enum Shaders { SKYBOX, BASIC_COLOUR_SHADER, BASIC_TEXTURE_SHADER, LIGHT_SHADER, LIGHT_TEXTURE_SHADER };
enum Textures { PLANE_TEXTURE, ASTEROID_TEXTURE };
GLfloat cameraSpeed = 0.005f;
GLfloat deltaTime = 1.0f / 60.0f;
GLfloat friction = 0.05f;
GLfloat lastX = 400, lastY = 300;
GLfloat resilience = 0.98f;
GLuint mode = AABB;
const GLuint numRigidBodies = 15;
GLuint shaderProgramID[NUM_SHADERS];
int screenWidth = 1000;
int screenHeight = 800;
int stringIDs[3];
//Model planeModel;
Mesh asteroid, boundingBox, sphereMesh;
vec4 red = vec4(1.0f, 0.0f, 0.0f, 1.0f);
vec4 green = vec4(0.0f, 1.0f, 0.0f, 1.0f);
vec4 yAxis = vec4(0.0f, 1.0f, 0.0f, 0.0f);
vec4 xAxis = vec4(1.0f, 0.0f, 0.0f, 0.0f);
vec4 zAxis = vec4(0.0f, 0.0f, 1.0f, 0.0f);
vector<RigidBody> rigidbodies;

// | Resource Locations
const char * meshFiles[NUM_MESHES] = { "../Meshes/plane.obj", "../Meshes/particle_reduced.dae", "../Meshes/particle.dae" };
const char * skyboxTextureFiles[6] = { "../Textures/DSposx.png", "../Textures/DSnegx.png", "../Textures/DSposy.png", "../Textures/DSnegy.png", "../Textures/DSposz.png", "../Textures/DSnegz.png"};
const char * textureFiles[NUM_TEXTURES] = { "../Textures/plane.jpg", "../Textures/asteroid.jpg"  };

const char * vertexShaderNames[NUM_SHADERS] = { "../Shaders/SkyboxVertexShader.txt", "../Shaders/ParticleVertexShader.txt", "../Shaders/BasicTextureVertexShader.txt", "../Shaders/LightVertexShader.txt", "../Shaders/LightTextureVertexShader.txt" };
const char * fragmentShaderNames[NUM_SHADERS] = { "../Shaders/SkyboxFragmentShader.txt", "../Shaders/ParticleFragmentShader.txt", "../Shaders/BasicTextureFragmentShader.txt", "../Shaders/LightFragmentShader.txt", "../Shaders/LightTextureFragmentShader.txt" };

string frf(const float &f)
{
	ostringstream ss;
	ss << setfill(' ') << std::setw(6) << fixed << setprecision(3) << f;
	string s(ss.str());
	return s;
}

void draw_text()
{
	ostringstream typeOSS, numOSS;
	string typeString, numString;
	if (mode == BOUNDING_SPHERES)
		typeOSS << "Broad Phase Collision: Bounding Spheres";
	else if (mode == AABB)
		typeOSS << "Broad Phase Collision: AABB";
	else
		typeOSS << "Broad Phase Collision: None";

	numOSS <<"Number of rigid bodies: " << numRigidBodies;
	//closestOSS << "Closest Point ( " << fixed << setprecision(3) << closestPoint.v[0] << ", " << closestPoint.v[1] << ", " << closestPoint.v[2] << " )";
	
	typeString = typeOSS.str();
	numString = numOSS.str();
	//closestString = closestOSS.str();
	
	update_text(stringIDs[0], typeString.c_str());
	update_text(stringIDs[1], numString.c_str());
	//update_text(stringIDs[2], closestString.c_str());

	draw_texts();
}

void init_text()
{
	stringIDs[0] = add_text("Broad Phase Collision: ", -0.95f, 0.95f, 25.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[1] = add_text("Number of rigid bodies: ", -0.95f, 0.9f, 25.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	//stringIDs[2] = add_text("Closest Point (,,)", -0.95f, 0.85f, 25.0f, 1.0f, 1.0f, 1.0f, 1.0f);
}

void display() 
{
	// Tell GL to only draw onto a pixel if the shape is closer to the viewer
	glEnable(GL_DEPTH_TEST);	// Enable depth-testing
	glDepthFunc(GL_LESS);		// Depth-testing interprets a smaller value as "closer"
	glClearColor(0.0f/255.0f, 0.0f/255.0f, 0.0f/255.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw skybox first
	mat4 view = camera.GetViewMatrix();
	mat4 projection = perspective(camera.Zoom, (float)screenWidth / (float)screenHeight, 0.1f, 100.0f);
	mat4 model = identity_mat4();
	vec4 view_position = vec4(camera.Position.v[0], camera.Position.v[1], camera.Position.v[2], 0.0f);

	//planeModel.drawModel(view, projection, model, vec4(0.0f, 0.0f, 0.0f, 0.0f), view_position);
	//asteroid.drawMesh(view, projection, model, vec4(0.0f, 0.0f, 0.0f, 0.0f), view_position);
	//sphereMesh.drawLine(view, projection, model, vec4(0.0f, 1.0f, 0.0f, 1.0f));

	boundingBox.drawLine(view, projection, model, vec4(1.0f, 1.0f, 0.0f, 1.0f));

	for (GLuint i = 0; i < numRigidBodies; i++)
	{
		rigidbodies[i].drawMesh(view, projection, view_position);
		if (mode == BOUNDING_SPHERES)
			rigidbodies[i].drawBoundingSphere(view, projection);
		else if (mode == AABB)
			rigidbodies[i].drawAABB(view, projection, &shaderProgramID[BASIC_COLOUR_SHADER]);
	}
	
	draw_text();
	
	glutSwapBuffers();
}

void processInput()
{
	if (keys[GLUT_KEY_UP])
		camera.ProcessKeyboard(FORWARD, cameraSpeed);
	if(keys[GLUT_KEY_DOWN])
		camera.ProcessKeyboard(BACKWARD, cameraSpeed);
	if (keys[GLUT_KEY_LEFT])
		camera.ProcessKeyboard(LEFT, cameraSpeed);
	if (keys[GLUT_KEY_RIGHT])
		camera.ProcessKeyboard(RIGHT, cameraSpeed);

	if (keys['1'])
		mode = BOUNDING_SPHERES;
	if (keys['2'])
		mode = AABB;
	if (keys['p'])
		pause = true;
	if (keys['o'])
		pause = false;

	if (keys[(char)27])
		exit(0);
}

void checkPlaneCollisions(RigidBody &rigidBody)
{
	if (pointToPlane(rigidBody.position, yAxis, (yAxis * -1.0f)) < rigidBody.boundingSphereRadius)
	{
		vec4 velocityNormal = yAxis * (dot(rigidBody.velocity, yAxis));
		vec4 velocityTangent = rigidBody.velocity - velocityNormal;
		rigidBody.linearMomentum = ((velocityTangent * (1 - friction)) - (velocityNormal * resilience)) * rigidBody.mass;
		rigidBody.position -= yAxis * (2 * (dot((rigidBody.position - (yAxis * -1.0f) - vec4(0.0f, rigidBody.boundingSphereRadius, 0.0f, 0.0f)), yAxis)));
	}
	if (pointToPlane(rigidBody.position, (yAxis * -1), (yAxis * 1.0f)) < rigidBody.boundingSphereRadius)
	{
		vec4 velocityNormal = (yAxis * -1) * (dot(rigidBody.velocity, (yAxis * -1)));
		vec4 velocityTangent = rigidBody.velocity - velocityNormal;
		rigidBody.linearMomentum = ((velocityTangent * (1 - friction)) - (velocityNormal * resilience)) * rigidBody.mass;
		rigidBody.position -= (yAxis * -1) * (2 * (dot((rigidBody.position - (yAxis * 1.0f) + vec4(0.0f, rigidBody.boundingSphereRadius, 0.0f, 0.0f)), (yAxis * -1))));
	}
	if (pointToPlane(rigidBody.position, xAxis, (xAxis * -1.0f)) < rigidBody.boundingSphereRadius)
	{
		vec4 velocityNormal = xAxis * (dot(rigidBody.velocity, xAxis));
		vec4 velocityTangent = rigidBody.velocity - velocityNormal;
		rigidBody.linearMomentum = ((velocityTangent * (1 - friction)) - (velocityNormal * resilience)) * rigidBody.mass;
		rigidBody.position -= xAxis * (2 * (dot((rigidBody.position - (xAxis * -1.0f) - vec4(rigidBody.boundingSphereRadius, 0.0f, 0.0f, 0.0f)), xAxis)));
	}
	if (pointToPlane(rigidBody.position, (xAxis * -1), (xAxis * 1.0f)) < rigidBody.boundingSphereRadius)
	{
		vec4 velocityNormal = (xAxis * -1) * (dot(rigidBody.velocity, (xAxis * -1)));
		vec4 velocityTangent = rigidBody.velocity - velocityNormal;
		rigidBody.linearMomentum = ((velocityTangent * (1 - friction)) - (velocityNormal * resilience)) * rigidBody.mass;
		rigidBody.position -= (xAxis * -1) * (2 * (dot((rigidBody.position - (xAxis * 1.0f) + vec4(rigidBody.boundingSphereRadius, 0.0f, 0.0f, 0.0f)), (xAxis * -1))));
	}
	if (pointToPlane(rigidBody.position, zAxis, (zAxis * -1.0f)) < rigidBody.boundingSphereRadius)
	{
		vec4 velocityNormal = zAxis * (dot(rigidBody.velocity, zAxis));
		vec4 velocityTangent = rigidBody.velocity - velocityNormal;
		rigidBody.linearMomentum = ((velocityTangent * (1 - friction)) - (velocityNormal * resilience)) * rigidBody.mass;
		rigidBody.position -= zAxis * (2 * (dot((rigidBody.position - (zAxis * -1.0f) - vec4(0.0f, 0.0f, rigidBody.boundingSphereRadius, 0.0f)), zAxis)));
	}
	if (pointToPlane(rigidBody.position, (zAxis * -1), (zAxis * 1.0f)) < rigidBody.boundingSphereRadius)
	{
		vec4 velocityNormal = (zAxis * -1) * (dot(rigidBody.velocity, (zAxis * -1)));
		vec4 velocityTangent = rigidBody.velocity - velocityNormal;
		rigidBody.linearMomentum = ((velocityTangent * (1 - friction)) - (velocityNormal * resilience)) * rigidBody.mass;
		rigidBody.position -= (zAxis * -1) * (2 * (dot((rigidBody.position - (zAxis * 1.0f) + vec4(0.0f, 0.0f, rigidBody.boundingSphereRadius, 0.0f)), (zAxis * -1))));
	}
}

void updateRigidBodies()
{
	for (GLuint i = 0; i < numRigidBodies; i++)
	{
		RigidBody &rigidBody = rigidbodies[i];
		// Might change this to user input
		//computeForcesAndTorque();

		rigidBody.position += rigidBody.velocity * deltaTime;

		// Check for collision
		checkPlaneCollisions(rigidBody);

		versor omega;
		omega.q[0] = 0.0f;
		omega.q[1] = rigidBody.angularVelocity.v[0];
		omega.q[2] = rigidBody.angularVelocity.v[1];
		omega.q[3] = rigidBody.angularVelocity.v[2];

		versor angularVelocityQuat;
		float avMag = quatMagnitude(omega);
		angularVelocityQuat.q[0] = cos((avMag * deltaTime) / 2);
		if (avMag > 0)
		{
			angularVelocityQuat.q[1] = (rigidBody.angularVelocity.v[0] / avMag) * sin((avMag * deltaTime) / 2);
			angularVelocityQuat.q[2] = (rigidBody.angularVelocity.v[1] / avMag) * sin((avMag * deltaTime) / 2);
			angularVelocityQuat.q[3] = (rigidBody.angularVelocity.v[2] / avMag) * sin((avMag * deltaTime) / 2);
		}
		else
		{
			angularVelocityQuat.q[1] = 0.0f;
			angularVelocityQuat.q[2] = 0.0f;
			angularVelocityQuat.q[3] = 0.0f;
		}

		multiplyQuat(rigidBody.orientation, angularVelocityQuat, rigidBody.orientation);

		rigidBody.linearMomentum += rigidBody.force * deltaTime;
		rigidBody.angularMomentum += rigidBody.torque * deltaTime;

		rigidBody.velocity = rigidBody.linearMomentum / rigidBody.mass;
		rigidBody.rotation = quat_to_mat4(normalise(rigidBody.orientation));
		rigidBody.Iinv = rigidBody.rotation * rigidBody.IbodyInv * transpose(rigidBody.rotation);
		rigidBody.angularVelocity = rigidBody.Iinv * rigidBody.angularMomentum;

		// Update all world points
		for (int i = 0; i < rigidBody.numPoints; i++)
		{
			rigidBody.worldVertices[i] = (rigidBody.rotation * rigidBody.initialWorldVertices[i]) + rigidBody.position;
		}

		if (mode == AABB)
		{
			rigidBody.xMin = rigidBody.worldVertices[0].v[0];
			rigidBody.xMax = rigidBody.worldVertices[0].v[0];
			rigidBody.yMin = rigidBody.worldVertices[0].v[1];
			rigidBody.yMax = rigidBody.worldVertices[0].v[1];
			rigidBody.zMin = rigidBody.worldVertices[0].v[2];
			rigidBody.zMax = rigidBody.worldVertices[0].v[2];

			for (int i = 1; i < rigidBody.numPoints; i++)
			{
				vec4 vertex = rigidBody.worldVertices[i];

				if (vertex.v[0] < rigidBody.xMin)
					rigidBody.xMin = vertex.v[0];
				else if (vertex.v[0] > rigidBody.xMax)
					rigidBody.xMax = vertex.v[0];

				if (vertex.v[1] < rigidBody.yMin)
					rigidBody.yMin = vertex.v[1];
				else if (vertex.v[1] > rigidBody.yMax)
					rigidBody.yMax = vertex.v[1];

				if (vertex.v[2] < rigidBody.zMin)
					rigidBody.zMin = vertex.v[2];
				else if (vertex.v[2] > rigidBody.zMax)
					rigidBody.zMax = vertex.v[2];
			}
		}

		// Reset the colliding with counter
		rigidBody.collidingWith = 0;
	}
}

void checkBoundingSphereCollisions()
{
	for (GLuint i = 0; i < numRigidBodies; i++)
	{
		RigidBody &rb1 = rigidbodies[i];

		for (GLuint j = i + 1; j < numRigidBodies; j++)
		{
			RigidBody &rb2 = rigidbodies[j];

			if (getDistance(rb1.position, rb2.position) <= (rb1.boundingSphereRadius + rb2.boundingSphereRadius))
			{
				rb1.collidingWith++;
				rb2.collidingWith++;
				rb1.boundingSphereColour = red;
				rb2.boundingSphereColour = red;
			}
		}

		if (rb1.collidingWith == 0)
		{
			rb1.boundingSphereColour = green;
		}
	}
}

bool isColliding(const RigidBody& bdi, const RigidBody& cdi)
{
	if ((bdi.xMax < cdi.xMin || bdi.xMin > cdi.xMax)) return false;
	if ((bdi.yMax < cdi.yMin || bdi.yMin > cdi.yMax)) return false;
	if ((bdi.zMax < cdi.zMin || bdi.zMin > cdi.zMax)) return false;

	return true;
}

void checkAABBCollisions()
{
	vector<bool> m_collision(numRigidBodies, false);
	for (int i = 0; i < numRigidBodies; i++)
	{
		for (int j = i + 1; j < numRigidBodies; j++)
		{
			if (isColliding(rigidbodies[i], rigidbodies[j]))
			{
				m_collision[i] = true;
				m_collision[j] = true;
			}
		}
	}

	for (int k = 0; k < numRigidBodies; k++)
	{
		rigidbodies[k].collisionAABB = m_collision[k];
		rigidbodies[k].boundingBoxColour = m_collision[k] ? red : green;
	}
}

void updateScene()
{
	processInput();

	if (!pause)
	{
		updateRigidBodies();

		if (mode == BOUNDING_SPHERES)
			checkBoundingSphereCollisions();
		else if (mode == AABB)
			checkAABBCollisions();
	}

	// Draw the next frame
	glutPostRedisplay();
}

void initialiseRigidBodies()
{
	for (GLuint i = 0; i < numRigidBodies; i++)
	{
		RigidBody &rigidBody = rigidbodies[i];
		GLfloat randomX1 = ((rand() % 10) - 5) / 100000.0f;
		GLfloat randomY1 = ((rand() % 10) - 5) / 100000.0f;
		GLfloat randomZ1 = ((rand() % 10) - 5) / 100000.0f;
		GLfloat randomX2 = ((rand() % 100) - 50) / 50000.0f;
		GLfloat randomY2 = ((rand() % 100) - 50) / 50000.0f;
		GLfloat randomZ2 = ((rand() % 100) - 50) / 50000.0f;
		rigidBody.angularMomentum = vec4(randomX1, randomY1, randomZ1, 0.0f);
		rigidBody.linearMomentum = vec4(randomX2, randomY2, randomZ2, 0.0f);

		rigidBody.xMinI = 2 * i;
		rigidBody.xMaxI = (2 * i) + 1;
		rigidBody.yMinI = 2 * i;
		rigidBody.yMaxI = (2 * i) + 1;
		rigidBody.zMinI = 2 * i;
		rigidBody.zMaxI = (2 * i) + 1;
	}
}

void init()
{
	if (!init_text_rendering("../Textures/freemono.png", "../Textures/freemono.meta", screenWidth, screenHeight))
	{
		fprintf(stderr, "ERROR init text rendering\n");
		exit(1);
	}
	init_text();

	// Compile the shaders
	for (int i = 0; i < NUM_SHADERS; i++)
	{
		shaderProgramID[i] = CompileShaders(vertexShaderNames[i], fragmentShaderNames[i]);
	}


	GLfloat bounding_box_vertices[] = {
		1.0f, 1.0f, 1.0f,
		-1.0f, 1.0f, 1.0f,
		-1.0f, -1.0f, 1.0f,
		1.0f, -1.0f, 1.0f,

		1.0f, 1.0f, 1.0f,
		1.0f, 1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, 1.0f,

		-1.0f, -1.0f, 1.0f,
		-1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, 1.0f, -1.0f,

		-1.0f, 1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f, 1.0f, -1.0f,
		-1.0f, 1.0f, 1.0f
	};

	boundingBox = Mesh(&shaderProgramID[BASIC_COLOUR_SHADER]);
	boundingBox.generateObjectBufferMesh(bounding_box_vertices, 16);

	//planeModel = Model(&shaderProgramID[LIGHT_TEXTURE_SHADER], meshFiles[PLANE_MESH], textureFiles[PLANE_TEXTURE]);
	asteroid = Mesh(&shaderProgramID[LIGHT_TEXTURE_SHADER]);
	asteroid.generateObjectBufferMesh(meshFiles[ASTEROID_MESH]);
	asteroid.loadTexture(textureFiles[ASTEROID_TEXTURE]);

	sphereMesh = Mesh(&shaderProgramID[BASIC_COLOUR_SHADER]);
	sphereMesh.generateObjectBufferMesh(meshFiles[SPHERE_MESH]);

	//RigidBody rigidBody = RigidBody(asteroid.vertex_count, asteroid.vertex_positions);
	RigidBody rigidBody = RigidBody(asteroid, 0.2f);
	rigidBody.addBoundingSphere(sphereMesh, green);
	//rigidBody.scaleFactor = 0.2f;

	for (GLuint i = 0; i < numRigidBodies; i++)
		rigidbodies.push_back(rigidBody);

	initialiseRigidBodies();
}

/*
 *	User Input Functions
 */
#pragma region USER_INPUT_FUNCTIONS
void pressNormalKeys(unsigned char key, int x, int y)
{
	keys[key] = true;
}

void releaseNormalKeys(unsigned char key, int x, int y)
{
	keys[key] = false;
}

void pressSpecialKeys(int key, int x, int y)
{
	keys[key] = true;
}

void releaseSpecialKeys(int key, int x, int y)
{
	keys[key] = false;
}

void mouseClick(int button, int state, int x, int y)
{}

void processMouse(int x, int y)
{
	if (firstMouse)
	{
		lastX = x;
		lastY = y;
		firstMouse = false;
	}

	GLfloat xoffset = x - lastX;
	GLfloat yoffset = lastY - y;

	lastX = x;
	lastY = y;

	camera.ProcessMouseMovement(xoffset, yoffset);
}

void mouseWheel(int button, int dir, int x, int y)
{}
#pragma endregion

/*
 *	Main
 */
int main(int argc, char** argv) 
{
	srand(time(NULL));

	// Set up the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(screenWidth, screenHeight);
	glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - screenWidth) / 2, (glutGet(GLUT_SCREEN_HEIGHT) - screenHeight) / 4);
	glutCreateWindow("Distance and Contact");

	// Glut display and update functions
	glutDisplayFunc(display);
	glutIdleFunc(updateScene);

	// User input functions
	glutKeyboardFunc(pressNormalKeys);
	glutKeyboardUpFunc(releaseNormalKeys);
	glutSpecialFunc(pressSpecialKeys);
	glutSpecialUpFunc(releaseSpecialKeys);
	glutMouseFunc(mouseClick);
	glutPassiveMotionFunc(processMouse);
	glutMouseWheelFunc(mouseWheel);


	glewExperimental = GL_TRUE; //for non-lab machines, this line gives better modern GL support
	
	// A call to glewInit() must be done after glut is initialized!
	GLenum res = glewInit();
	// Check for any errors
	if (res != GLEW_OK) {
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
		return 1;
	}

	// Set up meshes and shaders
	init();
	// Begin infinite event loop
	glutMainLoop();
	return 0;
}