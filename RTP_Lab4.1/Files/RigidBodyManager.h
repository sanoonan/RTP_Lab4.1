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


#include "RigidBody.h"
#include "CollisionPair.h"
#include "AntTweakBar.h"
#include "ContactModel.h"

using namespace std;

class RigidBodyManager
{
public:

	RigidBodyManager();

	float coeffRestitution;

	std::vector<RigidBody> bodies;
	
	string collision_method;

	int num;

	float drag_coeff;

	void addRigidBody(RigidBody body);
	void addRigidBody(Mesh mesh);
	void addRigidBody(const char* filename);


	void addTBar(TwBar *bar);

	void load_mesh();
	void draw(GLuint spID);



	void update(float dt);

	void reset();

	bool checkRayHit(glm::vec3 ray_origin, glm::vec3 p1, glm::vec3 p2, int &hit_target, glm::vec3 &hit_pos);

	void drawCollisionBoxes(GLuint spID);
	void drawBSpheres(GLuint spID);
	void drawAABBs(GLuint spID);

	void checkCollisionsSphere();
	void checkCollisionsAABB();

	struct SPpoint
	{
		int id;
		float point;
		bool start;

		SPpoint *prev;
		SPpoint *next;

		SPpoint()
		{
			id = NULL;
			point = NULL;
			start = false;
			prev = NULL;
			next = NULL;
		}

		SPpoint(int _id, float _point, bool _start)
		{
			id = _id;
			point = _point;
			start = _start;
			prev = NULL;
			next = NULL;
		}

		bool operator > (SPpoint &other)
		{
			return point > other.point;
		}

		bool operator< (SPpoint &other)
		{
			return point < other.point;
		}	
	};

	struct SPpointList
	{
		SPpoint *head;
		SPpoint *tail;

		SPpointList()
		{
			head = NULL;
			tail = NULL;
		}

		void addToEnd(SPpoint &p)
		{
			if(head == NULL)
			{
				head = &p;
				tail = &p;
				p.prev = NULL;
				p.next = NULL;
			}
			else
			{
				tail->next = &p;
				p.prev = tail;
				p.next = NULL;
				tail = &p;
			}
		}

		void addToStart(SPpoint &p)
		{
			if(head == NULL)
			{
				head = &p;
				tail = &p;
				p.prev = NULL;
				p.next = NULL;
			}
			else
			{
				head->prev = &p;
				p.prev = NULL;
				p.next = head;
				head = &p;
			}
		}
	};
	
	std::vector<SPpoint> unsortedx;
	std::vector<SPpoint> unsortedy;
	std::vector<SPpoint> unsortedz;

	SPpointList xlist;
	SPpointList ylist;
	SPpointList zlist;


	void createUSlists();
	void updateUSlists();
	void createSPlists();
	void updateSPlists();
	void insertionSort(SPpointList &list);

	void checkCollisionsAABBSweepPrune();


	std::vector<CollisionPair> broad_collision_pairs;
	std::vector<CollisionPair> narrow_collision_pairs;

	void clearCollisions();
	void updateCollisions();

	void getMinkowskiDiff(std::vector<glm::vec3> &diff_verts, std::vector<glm::vec3> &v1, std::vector<glm::vec3> &v2);
	glm::vec3 supportMap(std::vector<glm::vec3> &verts, glm::vec3 dir);

	
	
	std::vector<glm::vec3> getJohnsons(std::vector<glm::vec3> &verts, glm::vec3 &P);
	std::vector<glm::vec3> getJohnsonsPlane(std::vector<glm::vec3> &simplex, glm::vec3 &P);

	

	glm::vec3 singleGJK(std::vector<glm::vec3> &verts, std::vector<glm::vec3> &sim);
	glm::vec3 doubleGJK(std::vector<glm::vec3> &verts, std::vector <glm::vec3> &a, std::vector<glm::vec3> &b);

	float GJK(CollisionPair pair);

	float collide_distance;

	
	bool isSameDirection(glm::vec3 v1, glm::vec3 v2);

	glm::vec3 diffSupportMap(std::vector<glm::vec3> &verts1, std::vector<glm::vec3> &verts2, glm::vec3 dir);
	glm::vec3 diffSupportMap(CollisionPair pair, glm::vec3 dir);
	glm::vec3 diffSupportMap(CollisionPair pair, glm::vec3 dir, glm::vec3 &v1, glm::vec3 &v2);


	glm::vec3 diffSupportMap(std::vector<glm::vec3> &verts1, std::vector<glm::vec3> &verts2, glm::vec3 dir, glm::vec3 &v1, glm::vec3 &v2);

	bool epaGJK(CollisionPair pair, std::vector<glm::vec3> &sim);

	float EPA(CollisionPair pair, std::vector<glm::vec3> &simplex, glm::vec3 &coll_normal);




	std::vector<glm::vec3> getJohnsonsPlaneIgnoreFace(std::vector<glm::vec3> &simplex, std::vector<std::vector<glm::vec3>> &faces, glm::vec3 &P);

	bool checkForVertex(glm::vec3 p,  std::vector<glm::vec3> &simplex, size_t sim_size);
	bool checkSubSimplex(std::vector<glm::vec3> &sub, std::vector<glm::vec3> &sim);
	void cullSimplices(std::vector<std::vector<glm::vec3>> &simplices, std::vector<std::vector<glm::vec3>> &faces);

	std::vector<std::vector<glm::vec3>> splitTetrahedron(std::vector<glm::vec3> &tet);
	int getJohnsonsPlane(std::vector<std::vector<glm::vec3>> &faces, glm::vec3 &P);

	float optEPA(CollisionPair piar, std::vector<glm::vec3> simplex, glm::vec3 &coll_normal);

	bool checkRayTriangleIntersection(glm::vec3 TP1, glm::vec3 TP2, glm::vec3 TP3, glm::vec3 LP1, glm::vec3 LP2, glm::vec3 &HitPos);
	std::vector<int> seenFaces(std::vector<std::vector<glm::vec3>> &faces, glm::vec3 point);
	void expandPoly(std::vector<std::vector<glm::vec3>> &faces, glm::vec3 spoint);

	bool checkForEdge(std::vector<glm::vec3> new_edge, std::vector<std::vector<glm::vec3>> &edges, int &num);

	bool checkForIntInVec(int num, std::vector<int> &vec);

	bool checkRayTri(std::vector<glm::vec3> &tri, glm::vec3 source, glm::vec3 dest, glm::vec3 &hitPos);

	bool checkForRepeat(glm::vec3 v, std::vector<glm::vec3> &verts);

	glm::vec3 getBarycentricCoords(glm::vec3 p, std::vector<glm::vec3> &tri);
	
	
	
	//void splitTetrahedron(std::vector<glm::vec3[3]> &faces, std::vector<glm::vec3> &tet);

	float EPA_GJK(CollisionPair pair, glm::vec3 &coll_normal, glm::vec3 &pointA, glm::vec3 &pointB, std::vector<glm::vec3> &a_tri, std::vector<glm::vec3> &b_tri);
	bool EPA_GJK(CollisionPair pair);

	struct SupportPoint
	{
		glm::vec3 sa;
		glm::vec3 sb;
		glm::vec3 sc;

		SupportPoint()
		{
			sa = sb = sc = glm::vec3();
		}

		SupportPoint(glm::vec3 c, glm::vec3 a, glm::vec3 b)
		{
			sa = a;
			sb = b;
			sc = c;
		}
		
		SupportPoint(glm::vec3 a, glm::vec3 b)
		{
			sa = a;
			sb = b;
			sc = a-b;
		}
	};


	bool epaGJK(CollisionPair pair, std::vector<SupportPoint> &sim);

	void diffSupportMap(CollisionPair pair, glm::vec3 dir, SupportPoint &sp);
	
	void getJohnsons(std::vector<SupportPoint> &result_simplex, std::vector<SupportPoint> &simplex, glm::vec3 &P);
	void getJohnsonsPlane(std::vector<SupportPoint> &plane, std::vector<SupportPoint> &simplex, glm::vec3 &P);

	float EPA(CollisionPair pair, std::vector<SupportPoint> &simplex, glm::vec3 &coll_normal, glm::vec3 &coll_point);
	bool checkForVertex(glm::vec3 p, std::vector<SupportPoint> &simplex, size_t sim_size);
	void getJohnsonsPlaneIgnoreFace(std::vector<SupportPoint> &result, std::vector<SupportPoint> &simplex, std::vector<std::vector<SupportPoint>> &faces, glm::vec3 &P);
	void cullSimplices(std::vector<std::vector<SupportPoint>> &simplices, std::vector<std::vector<SupportPoint>> &faces);
	bool checkSubSimplex(std::vector<SupportPoint> &sub, std::vector<SupportPoint> &sim);

	bool do_narrow_phase;

	void adjustDragCoeff();
};
