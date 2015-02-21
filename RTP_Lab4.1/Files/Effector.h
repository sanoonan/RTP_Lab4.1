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

#include "AntTweakBar.h"

class Effector
{
public:
	glm::vec3 position;
	glm::vec3 force_dir;
	float force_mag;
	float time;

	Effector();
	void addTBar(TwBar *bar);

	void setForceDir(glm::vec3 new_fd);
	glm::vec3 getForceDir() const;



	static void TW_CALL setForceDirCB(const void *value, void *clientData)
	{
		static_cast<Effector *>(clientData)->setForceDir(*static_cast<const glm::vec3 *>(value));
	}

	static void TW_CALL getForceDirCB(void *value, void *clientData)
    {
		
		*static_cast<glm::vec3 *>(value) = static_cast<const Effector *>(clientData)->getForceDir();
    }

};