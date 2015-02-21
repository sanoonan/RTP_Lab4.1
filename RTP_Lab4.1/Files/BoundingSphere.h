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

using namespace std;

class BoundingSphere
{
public:
	glm::vec3 centre;
	float radius;

	glm::vec3 transformed_centre;

	BoundingSphere();
	BoundingSphere(glm::vec3 _centre, float _radius);
	BoundingSphere(std::vector<glm::vec3> v);

	bool checkCollision(BoundingSphere other);


	void draw(GLuint spID, glm::vec3 colour);

	void update(glm::mat4 model_mat);


};