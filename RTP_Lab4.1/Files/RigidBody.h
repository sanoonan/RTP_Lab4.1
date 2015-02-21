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
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtc/constants.hpp>

#include "Mesh.h"
#include "Effector.h"
#include "BoundingSphere.h"
#include "AABB.h"

#include "AntTweakBar.h"

class RigidBody
{
public:

	int id;


	glm::vec3 position;
	glm::vec3 orientation;

	glm::vec3 scale;

	glm::vec3 velocity;
	glm::vec3 ang_velocity;


	glm::mat4 translation_mat;
	glm::mat3 rotation_mat;

	glm::mat4 model_mat;

	struct force_instance
	{
		glm::vec3 force, torque;
		float time_left;

		force_instance() {force = torque = glm::vec3(0.0f); time_left = 0;}
	};

	std::vector<force_instance> force_instances;
	force_instance drag;



	glm::mat3 inertial_tensor;
	glm::vec3 original_com;
	glm::vec3 centre_of_mass;

	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> transformed_vertices;

	float mass;

	float drag_coeff;

	Mesh mesh;

	BoundingSphere b_sphere;
	AABB aabb;

	bool collision;
	glm::vec3 no_hit_col, hit_col, current_col;

	RigidBody();
	RigidBody(Mesh _mesh);
	RigidBody(const char* filename);


	void load_mesh();
	void draw(GLuint spID);

	void updateBB();
	void drawBSphere(GLuint spID);
	void drawAABB(GLuint spID);

	void addTBar(TwBar *bar);

	void update(float dt);

	void updateTranslation(float dt);
	void updateRotation(float dt);


	void updateTranslationRK4(float dt);
	void updateRotationRK4(float dt);



	glm::vec3 calcDrag(glm::vec3 v);
	void updateDrag();
	glm::vec3 getOrientationFromRotMat(glm::mat3 mat);

	
	
	glm::mat3 makeAngVelMat(glm::vec3 av_vec);

	glm::mat3 calcInertialTensorBox();
	glm::mat3 calcMomentInertia(glm::mat3 rot_mat);

	void affectedByForce(Effector effector);

	glm::vec3 calcTotalForceWithDrag(std::vector<force_instance> fi);
	glm::vec3 calcTotalTorqueWithDrag(std::vector<force_instance> fi);
	glm::vec3 calcTotalForce(std::vector<force_instance> fi);
	glm::vec3 calcTotalTorque(std::vector<force_instance> fi);
	void cullInstances(std::vector<force_instance> &fi, float dt);

	void removeInstance(std::vector<force_instance> &fi, int num);

	void addDrag(float drag_coeff);

	void transformVertices();

	glm::vec3 updateCOM(std::vector<glm::vec3> v);

	void reset();

	bool checkRayHit(glm::vec3 ray_origin, glm::vec3 p1, glm::vec3 p2, glm::vec3 &hitPos);

	bool checkRayTriangleIntersection(glm::vec3 TP1, glm::vec3 TP2, glm::vec3 TP3, glm::vec3 LP1, glm::vec3 LP2, glm::vec3 &HitPos);

	
	std::vector<glm::vec3> reduced_verts;
	std::vector<glm::vec3> red_trans_verts;
	

	glm::vec3 colour;
};

