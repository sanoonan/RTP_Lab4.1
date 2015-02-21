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
#include <glm/gtx/perpendicular.hpp>


#include "RigidBodyManager.h"
#include "CollisionPair.h"
#include "AntTweakBar.h"

using namespace std;

class ContactModel
{
public:
	CollisionPair *pair;
	glm::vec3 pa, pb, norm;

	ContactModel();
		
	ContactModel(CollisionPair &_pair, glm::vec3 _pa, glm::vec3 _pb, glm::vec3 _norm);

	

	float calcImpulse(float coeff_rest);
	float calcLinearImpulse(float coeff_rest);
	float calcAngularImpulse(float coeff_rest);
};