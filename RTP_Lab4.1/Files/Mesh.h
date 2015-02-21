#pragma once

#define _CRT_SECURE_NO_DEPRECATE
//Some Windows Headers (For Time, IO, etc.)
#include <windows.h>
#include <mmsystem.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>


// Assimp includes

#include <assimp/cimport.h> // C importer
#include <assimp/scene.h> // collects data
#include <assimp/postprocess.h> // various extra operations
#include <stdio.h>
#include <math.h>
#include <vector> // STL dynamic memory.
#include <string>

#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4, glm::ivec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp> // glm::value_ptr

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

class Mesh
{
public:
	const char* filename;
	GLuint vao;

	int point_count;
	
	Mesh();
	Mesh(const char* _filename);

	bool load_mesh(std::vector<glm::vec3> &v, std::vector<glm::vec3> &r);
	void assign_vao(const aiScene* scene);
	void draw(GLuint spID, glm::mat4 model_mat);

	std::vector<glm::vec3> getVertices(std::vector<glm::vec3> &reduced);

	bool checkForRepeat(glm::vec3 v, std::vector<glm::vec3> &verts);

	void draw(GLuint spID, glm::mat4 model_mat, glm::vec3 colour);
};

