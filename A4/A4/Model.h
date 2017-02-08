#pragma once

// | Includes
#include <assimp/cimport.h>		// C importer
#include <assimp/Importer.hpp>
#include <assimp/scene.h>		// Collects data
#include <assimp/postprocess.h> // Various extra operations
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <math.h>
#include <mmsystem.h>
#include <stdio.h>
#include <vector>				// STL dynamic memory
#include <windows.h>

#include "Antons_maths_funcs.h" // Anton's maths functions
#include "Camera.h"
#include "Mesh.h"
#include "Shader_Functions.h"
#include "time.h"

using namespace std;

class Model {
public:
	Model();
	Model(GLuint* shaderID, const char* fileName, const char* texturePath);
	void drawModel(mat4 view, mat4 projection, mat4 model, vec4 colour, vec4 viewPosition);

private:
	GLuint shaderProgramID;
	string directory = "../Textures/";
	const char* texturePath;
	vector<Mesh> meshes;
	//vector<Texture> textures_loaded;


	void loadModel(string fileName);
	void processNode(aiNode* node, const aiScene* scene);
	//Mesh processMesh(aiMesh* mesh, const aiScene* scene);
	//vector<Texture> loadMaterialTextures(aiMaterial* mat, aiTextureType type, string typeName);
	//GLint loadTexture(const char* fileName, string directory);
};

Model::Model()
{}

Model::Model(GLuint* shaderID, const char* fileName, const char* texturePath = "")
{
	shaderProgramID = *shaderID;
	this->texturePath = texturePath;
	this->loadModel(fileName);
}

void Model::drawModel(mat4 view, mat4 projection, mat4 model, vec4 colour = vec4(0.0f, 0.0f, 0.0f, 0.0f), vec4 viewPosition = vec4(0.0f, 0.0f, 0.0f, 0.0f))
{
	for (GLuint i = 0; i < this->meshes.size(); i++)
		this->meshes[i].drawMesh(view, projection, model, colour, viewPosition);
}

/*  Functions   */
// Loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
void Model::loadModel(string fileName)
{
	// Read file via ASSIMP
	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(fileName, aiProcess_Triangulate | aiProcess_FlipUVs);
	// Check for errors
	if (!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
	{
		cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
		return;
	}

	printf("  %i animations\n", scene->mNumAnimations);
	printf("  %i cameras\n", scene->mNumCameras);
	printf("  %i lights\n", scene->mNumLights);
	printf("  %i materials\n", scene->mNumMaterials);
	printf("  %i meshes\n", scene->mNumMeshes);
	printf("  %i textures\n", scene->mNumTextures);

	// Process ASSIMP's root node recursively
	this->processNode(scene->mRootNode, scene);
}

// Processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
void Model::processNode(aiNode* node, const aiScene* scene)
{
	// Process each mesh located at the current node
	for (GLuint i = 0; i < node->mNumMeshes; i++)
	{
		// The node object only contains indices to index the actual objects in the scene. 
		// The scene contains all the data, node is just to keep stuff organized (like relations between nodes).
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		Mesh currentMesh = Mesh(&shaderProgramID);
		currentMesh.loadMesh(mesh, scene);
		currentMesh.generateObjectBufferMesh();
		currentMesh.loadTexture(this->texturePath);
		this->meshes.push_back(currentMesh);
	}
	// After we've processed all of the meshes (if any) we then recursively process each of the children nodes
	for (GLuint i = 0; i < node->mNumChildren; i++)
	{
		this->processNode(node->mChildren[i], scene);
	}

}

/*Mesh Model::processMesh(aiMesh* mesh, const aiScene* scene)
{
	// Data to fill
	vector<Vertex> vertices;
	vector<GLuint> indices;
	vector<Texture> textures;

	// Walk through each of the mesh's vertices
	for (GLuint i = 0; i < mesh->mNumVertices; i++)
	{
		Vertex vertex;
		vec3 vector; // We declare a placeholder vector since assimp uses its own vector class that doesn't directly convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
					 // Positions
		vector.v[0] = mesh->mVertices[i].x;
		vector.v[1] = mesh->mVertices[i].y;
		vector.v[2] = mesh->mVertices[i].z;
		vertex.position = vector;
		// Normals
		vector.v[0] = mesh->mNormals[i].x;
		vector.v[1] = mesh->mNormals[i].y;
		vector.v[2] = mesh->mNormals[i].z;
		vertex.normal = vector;
		// Texture Coordinates
		if (mesh->mTextureCoords[0]) // Does the mesh contain texture coordinates?
		{
			vec2 vec;
			// A vertex can contain up to 8 different texture coordinates. We thus make the assumption that we won't 
			// use models where a vertex can have multiple texture coordinates so we always take the first set (0).
			vec.v[0] = mesh->mTextureCoords[0][i].x;
			vec.v[1] = mesh->mTextureCoords[0][i].y;
			vertex.texture_coords = vec;
		}
		else
			vertex.texture_coords = vec2(0.0f, 0.0f);
		vertices.push_back(vertex);
	}
	// Now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
	for (GLuint i = 0; i < mesh->mNumFaces; i++)
	{
		aiFace face = mesh->mFaces[i];
		// Retrieve all indices of the face and store them in the indices vector
		for (GLuint j = 0; j < face.mNumIndices; j++)
			indices.push_back(face.mIndices[j]);
	}
	// Process materials
	if (mesh->mMaterialIndex >= 0)
	{
		aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
		// We assume a convention for sampler names in the shaders. Each diffuse texture should be named
		// as 'texture_diffuseN' where N is a sequential number ranging from 1 to MAX_SAMPLER_NUMBER. 
		// Same applies to other texture as the following list summarizes:
		// Diffuse: texture_diffuseN
		// Specular: texture_specularN
		// Normal: texture_normalN

		// 1. Diffuse maps
		vector<Texture> diffuseMaps = this->loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
		textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
		// 2. Specular maps
		vector<Texture> specularMaps = this->loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
		textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
	}

	// Return a mesh object created from the extracted mesh data
	return Mesh(vertices, indices, textures, shaderProgramID);
}

// Checks all material textures of a given type and loads the textures if they're not loaded yet.
// The required info is returned as a Texture struct.
vector<Texture> Model::loadMaterialTextures(aiMaterial* mat, aiTextureType type, string typeName)
{
	vector<Texture> textures;
	for (GLuint i = 0; i < mat->GetTextureCount(type); i++)
	{
		aiString str;
		mat->GetTexture(type, i, &str);
		// Check if texture was loaded before and if so, continue to next iteration: skip loading a new texture
		GLboolean skip = false;
		for (GLuint j = 0; j < textures_loaded.size(); j++)
		{
			if (textures_loaded[j].path == str)
			{
				textures.push_back(textures_loaded[j]);
				skip = true; // A texture with the same filepath has already been loaded, continue to next one. (optimization)
				break;
			}
		}
		if (!skip)
		{   // If texture hasn't been loaded already, load it
			Texture texture;
			texture.id = loadTexture(str.C_Str(), this->directory);
			texture.type = typeName;
			texture.path = str;
			textures.push_back(texture);
			this->textures_loaded.push_back(texture);  // Store it as texture loaded for entire model, to ensure we won't unnecesery load duplicate textures.
		}
	}
	return textures;
}

GLint Model::loadTexture(const char* fileName, string directory)
{
	GLuint textureID;
	string filename = string(fileName);
	filename = directory + fileName;

	if (this->texturePath != "")
	{
		filename = texturePath;
	}

	// Load texture data
	int width, height, n;
	unsigned char* image = stbi_load(fileName, &width, &height, &n, STBI_rgb);
	if (!image) {
		fprintf(stderr, "ERROR: could not load %s\n", fileName);
		return false;
	}
	// NPOT check
	if ((width & (width - 1)) != 0 || (height & (height - 1)) != 0) {
		fprintf(stderr, "WARNING: texture %s is not power-of-2 dimensions\n", fileName);
	}

	// Assign Texture to ID
	glGenTextures(1, &textureID);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
	glGenerateMipmap(GL_TEXTURE_2D);


	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	glBindTexture(GL_TEXTURE_2D, 0);

	stbi_image_free(image);

	return textureID;
}*/