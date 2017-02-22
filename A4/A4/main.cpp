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
#include "RigidBody.h"
#include "Shader_Functions.h"
#include "text.h"
#include "time.h"

using namespace std;

#define NUM_MESHES   3
#define NUM_SHADERS	 5
#define NUM_TEXTURES 2

bool firstMouse = true;
bool keys[1024];
bool pause = false;
Camera camera(vec3(0.0f, 0.0f, 4.0f));
const GLuint numRigidBodies = 10;
enum Meshes { PLANE_MESH, ASTEROID_MESH, SPHERE_MESH };
enum Shaders { SKYBOX, BASIC_COLOUR_SHADER, BASIC_TEXTURE_SHADER, LIGHT_SHADER, LIGHT_TEXTURE_SHADER };
enum Textures { PLANE_TEXTURE, ASTEROID_TEXTURE };
GLfloat cameraSpeed = 0.005f;
GLuint lastX = 400, lastY = 300;
GLuint mode = AABB;
GLuint shaderProgramID[NUM_SHADERS];
int screenWidth = 1000;
int screenHeight = 800;
int stringIDs[3];
Mesh asteroid, boundingBox, sphereMesh;
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
	
	typeString = typeOSS.str();
	numString = numOSS.str();
	
	update_text(stringIDs[0], typeString.c_str());
	update_text(stringIDs[1], numString.c_str());

	draw_texts();
}

void init_text()
{
	stringIDs[0] = add_text("Broad Phase Collision: ", -0.95f, 0.95f, 25.0f, 1.0f, 1.0f, 1.0f, 1.0f);
	stringIDs[1] = add_text("Number of rigid bodies: ", -0.95f, 0.9f, 25.0f, 1.0f, 1.0f, 1.0f, 1.0f);
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
	if (keys['0'])
		mode = -1;
	if (keys['p'])
		pause = true;
	if (keys['o'])
		pause = false;

	if (keys[(char)27])
		exit(0);
}

void updateScene()
{
	processInput();

	if (!pause)
	{
		updateRigidBodies(mode, numRigidBodies, rigidbodies);

		if (mode == BOUNDING_SPHERES)
			checkBoundingSphereCollisions(numRigidBodies, rigidbodies);
		else if (mode == AABB)
			checkAABBCollisions(numRigidBodies, rigidbodies);
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

	asteroid = Mesh(&shaderProgramID[LIGHT_TEXTURE_SHADER]);
	asteroid.generateObjectBufferMesh(meshFiles[ASTEROID_MESH]);
	asteroid.loadTexture(textureFiles[ASTEROID_TEXTURE]);

	sphereMesh = Mesh(&shaderProgramID[BASIC_COLOUR_SHADER]);
	sphereMesh.generateObjectBufferMesh(meshFiles[SPHERE_MESH]);

	RigidBody rigidBody = RigidBody(asteroid, 0.2f);
	rigidBody.addBoundingSphere(sphereMesh, green);

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

	int xoffset = x - lastX;
	int yoffset = lastY - y;

	lastX = x;
	lastY = y;

	camera.ProcessMouseMovement((GLfloat)xoffset, (GLfloat)yoffset);
}

void mouseWheel(int button, int dir, int x, int y)
{}
#pragma endregion

/*
 *	Main
 */
int main(int argc, char** argv) 
{
	srand((unsigned int)time(NULL));

	// Set up the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(screenWidth, screenHeight);
	glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - screenWidth) / 2, (glutGet(GLUT_SCREEN_HEIGHT) - screenHeight) / 4);
	glutCreateWindow("Broad-Phase Collision Detection");

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