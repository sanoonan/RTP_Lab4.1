#include "RigidBodyManager.h"

using namespace std;



RigidBodyManager :: RigidBodyManager()
{
	coeffRestitution = 1.0f;
	num = 0;
	drag_coeff = 0;
	do_narrow_phase = false;
	collision_method = "SweepPrune";
}


void RigidBodyManager :: addRigidBody(RigidBody body)
{
	body.id = num;
	body.drag_coeff = drag_coeff;
	bodies.push_back(body);
	num++;
}

void RigidBodyManager :: addRigidBody(Mesh mesh)
{
	RigidBody body(mesh);
	addRigidBody(body);
}


void RigidBodyManager :: addRigidBody(const char* filename)
{
	RigidBody body(filename);
	addRigidBody(body);
}

void TW_CALL changeCollision(void *clientData)
{
	string current = static_cast<RigidBodyManager *>(clientData)->collision_method;
	string next;

	if(current == "Spheres")
		next = "AABBs";
	else if(current == "AABBs")
		next = "SweepPrune";
	else
		next = "Spheres";

	static_cast<RigidBodyManager *>(clientData)->collision_method = next;
}

void RigidBodyManager :: addTBar(TwBar *bar)
{
	TwAddVarRW(bar, "Narrow Phase", TW_TYPE_BOOLCPP, &do_narrow_phase, "");
	TwAddVarRO(bar, "Distance", TW_TYPE_FLOAT, &collide_distance, "");

	TwAddVarRO(bar, "Collision Method", TW_TYPE_STDSTRING, &collision_method, "");
	TwAddButton(bar, "Change Method", changeCollision, this, "");
	TwAddVarRW(bar, "Restitution Coefficient", TW_TYPE_FLOAT, &coeffRestitution, "");
	TwAddVarRW(bar, "Drag Coefficient", TW_TYPE_FLOAT, &drag_coeff, "");
	TwAddVarRO(bar, "Number of Bodies", TW_TYPE_INT32, &num, "");
}

void RigidBodyManager :: load_mesh()
{
	for(int i=0; i<num; i++)
		bodies[i].load_mesh();

	createUSlists();
}

void RigidBodyManager :: createUSlists()
{
	int size = 0;
	for(int i=0; i<num; i++)
	{

		SPpoint p = SPpoint(bodies[i].id, bodies[i].aabb.vmin.x, true);
		unsortedx.push_back(p);

		unsortedx.push_back(SPpoint(bodies[i].id, bodies[i].aabb.vmax.x, false));

		unsortedy.push_back(SPpoint(bodies[i].id, bodies[i].aabb.vmin.y, true));

		unsortedy.push_back(SPpoint(bodies[i].id, bodies[i].aabb.vmax.y, false));

		unsortedz.push_back(SPpoint(bodies[i].id, bodies[i].aabb.vmin.z, true));

		unsortedz.push_back(SPpoint(bodies[i].id, bodies[i].aabb.vmax.z, false));
	}



	createSPlists();
}

void RigidBodyManager :: updateUSlists()
{
	for(int i=0; i<num*2; i++)
	{
		int id = unsortedx[i].id;
		bool start = unsortedx[i].start;
		if(start)
			unsortedx[i].point = bodies[id].aabb.vmin.x;
		else
			unsortedx[i].point = bodies[id].aabb.vmax.x;

		id = unsortedy[i].id;
		start = unsortedy[i].start;
		if(start)
			unsortedy[i].point = bodies[id].aabb.vmin.y;
		else
			unsortedy[i].point = bodies[id].aabb.vmax.y;

		id = unsortedz[i].id;
		start = unsortedz[i].start;
		if(start)
			unsortedz[i].point = bodies[id].aabb.vmin.z;
		else
			unsortedz[i].point = bodies[id].aabb.vmax.z;

	}
}

void RigidBodyManager :: createSPlists()
{
	for(int i=0; i<num*2; i++)
	{
		xlist.addToEnd(unsortedx[i]);
		ylist.addToEnd(unsortedy[i]);
		zlist.addToEnd(unsortedz[i]);
	}

	updateSPlists();
}

void RigidBodyManager :: updateSPlists()
{
	insertionSort(xlist);
	insertionSort(ylist);
	insertionSort(zlist);
}

void RigidBodyManager :: insertionSort(SPpointList &list)
{
	if(list.head == NULL)
		return;


	SPpointList new_list;
	SPpoint head = *list.head;

	new_list.addToStart(*list.head);

	SPpoint current_safe = *head.next;
	SPpoint *current = head.next;


	while(current != NULL)
	{
		SPpoint *check = new_list.tail;

		while(check != NULL)
		{
			if(current->point > check->point)
			{
				if(check->next != NULL)
				{
					current->next = check->next;
					check->next->prev = current;
				}
				else
				{
					current->next = NULL;
					new_list.tail = current;
				}

				current->prev = check;
				check->next = current;

				break;
			}
			else
				check = check->prev;
		}

		if(check == NULL)
			new_list.addToStart(*current);

		current = current_safe.next;

		if(current != NULL)
			current_safe = *current_safe.next;
	}

	list = new_list;
}



void RigidBodyManager :: draw(GLuint spID)
{
	for(int i=0; i<num; i++)
		bodies[i].draw(spID);
}


void RigidBodyManager :: update(float dt)
{
	adjustDragCoeff();

	for(int i=0; i<num; i++)
		bodies[i].update(dt);

	clearCollisions();

	if(collision_method == "Spheres")
		checkCollisionsSphere();
	else if (collision_method == "AABBs")
		checkCollisionsAABB();
	else
		checkCollisionsAABBSweepPrune();

	

	updateCollisions();

}



void RigidBodyManager :: reset()
{
	for(int i=0; i<num; i++)
		bodies[i].reset();
}


bool RigidBodyManager :: checkRayHit(glm::vec3 ray_origin, glm::vec3 p1, glm::vec3 p2, int &hit_target, glm::vec3 &hit_pos)
{
	float min_hit_dist;
	glm::vec3 min_hit_pos;
	int min_hit_target;
	bool hit = false;

	for(int i=0; i < num; i++)
	{
		glm::vec3 hit_position;
		if(bodies[i].checkRayHit(ray_origin, p1, p2, hit_position))
		{
			float hit_dist = glm::distance(ray_origin, hit_position);

			if(!hit)
			{
				hit = true;
				min_hit_pos = hit_position;
				min_hit_dist = hit_dist;
				min_hit_target = i;
			}
			else
			{
				if(hit_dist < min_hit_dist)
				{
					min_hit_dist = hit_dist;
					min_hit_pos = hit_position;
					min_hit_target = i;
				}
			}
		}
	}

	if(hit)
	{
		hit_target = min_hit_target;
		hit_pos = min_hit_pos;
	}

	return hit;
}


void RigidBodyManager :: drawCollisionBoxes(GLuint spID)
{
	if(collision_method == "Spheres")
		drawBSpheres(spID);
	else
		drawAABBs(spID);
}


void RigidBodyManager :: drawBSpheres(GLuint spID)
{
	for(int i=0; i<num; i++)
		bodies[i].drawBSphere(spID);
}

void RigidBodyManager :: drawAABBs(GLuint spID)
{
	for(int i=0; i<num; i++)
		bodies[i].drawAABB(spID);
}


void RigidBodyManager :: checkCollisionsSphere()
{


	for(int i=0; i<num; i++)
		for(int j=0; j<num; j++)
		{
			if(i != j)
			{
				if(bodies[i].b_sphere.checkCollision(bodies[j].b_sphere))
					broad_collision_pairs.push_back(CollisionPair(bodies[i], bodies[j]));
			}
		}
}

void RigidBodyManager :: checkCollisionsAABB()
{


	for(int i=0; i<num; i++)
		for(int j=i+1; j<num; j++)
		{

			if(bodies[i].aabb.checkCollision(bodies[j].aabb))
				broad_collision_pairs.push_back(CollisionPair(bodies[i], bodies[j]));

		}
}


void RigidBodyManager :: checkCollisionsAABBSweepPrune()
{
	updateUSlists();
	updateSPlists();


	std::vector<std::vector<int>> pairs(num);
	for(int i=0; i<num; i++)
		pairs[i].resize(num);

	for(int i=0; i<num; i++)
		for(int j=0; j<num; j++)
			pairs[i][j] = 0;


	std::vector<int> active_list;

	SPpoint *current = xlist.head;
	int num_active = 0;

	while(current != NULL)
	{
		int id = current->id;
		if(current->start)
		{
			active_list.push_back(id);

			if(active_list.size() > 1)
			{
				for(int i=0; i<active_list.size()-1; i++)
				{
					pairs[id][active_list[i]]++;
					pairs[active_list[i]][id]++;
				}
			}
		}
		else
		{
			for(int i=0; i<active_list.size(); i++)
				if(active_list[i] == id)
					active_list.erase(active_list.begin()+i);
		}


		if(current == xlist.tail)
			current = ylist.head;
		else if(current == ylist.tail)
			current = zlist.head;
		else
			current = current->next;
	}


	for(int i=0; i<num; i++)
		for(int j=0; j<num; j++)
			if(pairs[i][j] >= 3)
				broad_collision_pairs.push_back(CollisionPair(bodies[i], bodies[j]));
}

void RigidBodyManager :: clearCollisions()
{
	for(int i=0; i<num; i++)
	{
		bodies[i].collision = false;
		bodies[i].colour = glm::vec3(0.7f);
	}

	broad_collision_pairs.clear();
	narrow_collision_pairs.clear();

	
}

void RigidBodyManager :: updateCollisions()
{
	int broad_size = broad_collision_pairs.size();

	if(do_narrow_phase)
		for(int i=0; i<broad_size; i++)
			if(EPA_GJK(broad_collision_pairs[i]))
				narrow_collision_pairs.push_back(broad_collision_pairs[i]);

	size_t narrow_size = narrow_collision_pairs.size();

	for(int i=0; i<broad_size; i++)
	{
		broad_collision_pairs[i].body1->collision = true;
		broad_collision_pairs[i].body2->collision = true;
	}

	for(int i=0; i<narrow_size; i++)
	{
		narrow_collision_pairs[i].body1->colour = glm::vec3(0.7f, 0.0f, 0.0f);
		narrow_collision_pairs[i].body2->colour = glm::vec3(0.7f, 0.0f, 0.0f);
	}


}



void RigidBodyManager :: getMinkowskiDiff(std::vector<glm::vec3> &diff_verts, std::vector<glm::vec3> &v1, std::vector<glm::vec3> &v2)
{
	int isize = v1.size();
	int jsize = v2.size();
	int num = 0;
	diff_verts.resize(isize*jsize);

	for(int i=0; i<isize; i++)
		for(int j=0; j<jsize; j++)
		{
			glm::vec3 v = v1[i] - v2[j];
			diff_verts[num] = v;
			num++;
		}

}


glm::vec3 RigidBodyManager :: supportMap(std::vector<glm::vec3> &verts, glm::vec3 dir)
{
	glm::vec3 p;

	float max_dot_prod = glm::dot(verts[0], dir);
	p = verts[0];

	for(int i=1; i<verts.size(); i++)
	{
		float dot_prod = glm::dot(verts[i], dir);
		if(dot_prod > max_dot_prod)
		{
			max_dot_prod = dot_prod;
			p = verts[i];
		}
	}

	return p;
}



std::vector<glm::vec3> RigidBodyManager :: getJohnsons(std::vector<glm::vec3> &verts, glm::vec3 &P)
{
	std::vector<glm::vec3> vs;



	for(int i=0; i<verts.size(); i++)
	{
		bool closest_point = true;
		glm::vec3 a = verts[i];
		glm::vec3 ax = -a;

		for(int j=0; j<verts.size(); j++)
			if(i != j)
			{
				glm::vec3 b = verts[j];


				glm::vec3 ab = b-a;

				float ax_dot_ab = glm::dot(ax, ab);
				if(ax_dot_ab > 0)
				{
					closest_point = false;
					break;
				}
			}

			if(closest_point)
			{
				P = a;
				vs.push_back(a);
				return vs;
			}
	}


	for(int i=0; i<verts.size()-1; i++)
		for(int j=i+1; j<verts.size(); j++)
		{
			bool closest_edge = true;

			glm::vec3 a = verts[i];
			glm::vec3 b = verts[j];

			if(a == b)
			{
				closest_edge = false;
				break;
			}

			glm::vec3 ax = -a;
			glm::vec3 bx = -b;

			glm::vec3 ab = b-a;
			glm::vec3 ba = a-b;

			float ax_dot_ab = glm::dot(ax, ab);
			if(ax_dot_ab < 0)
			{
				closest_edge = false;
				break;
			}

			float bx_dot_ba = glm::dot(bx, ba);
			if(bx_dot_ba < 0)
			{
				closest_edge = false;
				break;
			}


			for(int k=0; k<verts.size(); k++)
				if((k != i)&&(k != j))
				{
					glm::vec3 c = verts[k];

					glm::vec3 bc = c-b;

					glm::vec3 cross = glm::cross(bc, ba);
					cross = glm::cross(cross, ba);

					float cross_dot_bx = glm::dot(cross, bx);
					if(cross_dot_bx < 0)
					{
						closest_edge = false;
						break;
					}
				}


				if(closest_edge)
				{

					glm::vec3 n_ab = glm::normalize(ab);
					float ax_dot_nab = glm::dot(ax, n_ab);

					P = a + ax_dot_nab*n_ab;



					vs.push_back(a);
					vs.push_back(b);
					return vs;
				}
		}

		vs = getJohnsonsPlane(verts, P);


		return vs;
}


std::vector<glm::vec3> RigidBodyManager :: getJohnsonsPlane(std::vector<glm::vec3> &simplex, glm::vec3&P)
{
	bool got_min = false;
	float min_dist = 0.0f;
	std::vector<glm::vec3> vs;

	for(int i=0; i<simplex.size(); i++)
		for(int j=i+1; j<simplex.size(); j++)
			for(int k=j+1; k<simplex.size(); k++)
			{
				//		if((i==j)||(i==k)||(j==k))
				//			break;


				glm::vec3 a = simplex[i];
				glm::vec3 b = simplex[j];
				glm::vec3 c = simplex[k];

				glm::vec3 ab = b-a;
				glm::vec3 ac = c-a;

				glm::vec3 norm = glm::cross(ab, ac);

				glm::vec3 n_norm = norm;
				if(norm != glm::vec3(0.0f))
					n_norm = glm::normalize(norm);

				float this_dist = glm::dot(n_norm, -a);

				if((abs(this_dist) < abs(min_dist))||(!got_min))
				{


					glm::vec3 p = -this_dist*n_norm;

					glm::vec3 vmin, vmax;
					vmin = vmax = a;

					if(b.x < vmin.x)
						vmin.x = b.x;
					if(b.y < vmin.y)
						vmin.y = b.y;
					if(b.z < vmin.z)
						vmin.z = b.z;

					if(b.x > vmax.x)
						vmax.x = b.x;
					if(b.y > vmax.y)
						vmax.y = b.y;
					if(b.z > vmax.z)
						vmax.z = b.z;


					if(c.x < vmin.x)
						vmin.x = c.x;
					if(c.y < vmin.y)
						vmin.y = c.y;
					if(c.z < vmin.z)
						vmin.z = c.z;

					if(c.x > vmax.x)
						vmax.x = c.x;
					if(c.y > vmax.y)
						vmax.y = c.y;
					if(c.z > vmax.z)
						vmax.z = c.z;


					if(((p.x <= vmax.x)&&(p.x >= vmin.x)&&(p.y <= vmax.y)&&(p.y >= vmin.y)&&(p.z <= vmax.z)&&(p.z >= vmin.z))||(!got_min))
					{
						got_min = true;
						vs.clear();
						vs.push_back(a);
						vs.push_back(b);
						vs.push_back(c);
						min_dist = this_dist;
						P = p;
					}
				}
			}



			return vs;
}



bool RigidBodyManager :: isSameDirection(glm::vec3 v1, glm::vec3 v2)
{
	return (glm::dot(v1, v2) > 0);
}

glm::vec3 RigidBodyManager :: diffSupportMap(std::vector<glm::vec3> &verts1, std::vector<glm::vec3> &verts2, glm::vec3 dir)
{
	glm::vec3 sa = supportMap(verts1, dir);
	glm::vec3 sb = supportMap(verts2, -dir);
	glm::vec3 sc = sa - sb;
	return sc;
}



glm::vec3 RigidBodyManager :: diffSupportMap(std::vector<glm::vec3> &verts1, std::vector<glm::vec3> &verts2, glm::vec3 dir, glm::vec3 &p1, glm::vec3 &p2)
{
	glm::vec3 sa = supportMap(verts1, dir);
	glm::vec3 sb = supportMap(verts2, -dir);
	glm::vec3 sc = sa - sb;
	p1 = sa;
	p2 = sb;
	return sc;
}



bool RigidBodyManager :: epaGJK(CollisionPair pair, std::vector<glm::vec3> &sim)
{
	std::vector<glm::vec3> simplex;

	glm::vec3 P = pair.body1->red_trans_verts[0] - pair.body2->red_trans_verts[0];


	simplex.push_back(P);

	bool intersection = false;

	int num = 0;
	while(num < 100)
	{
		simplex = getJohnsons(simplex, P);




		if(P == glm::vec3(0.0f))
			break;



		glm::vec3 n_p = glm::normalize(-P);

		glm::vec3 V = diffSupportMap(pair.body1->red_trans_verts, pair.body2->red_trans_verts, -P);



		float v_dot = glm::dot(V, n_p);
		float p_dot = glm::dot(P, n_p);

		if(v_dot <= p_dot+0.05f)
			return false;


		simplex.push_back(V);

		num++;
	}
	num = 0;


	while(simplex.size() < 4)
	{
		glm::vec3 p1 = pair.body1->red_trans_verts[num] - pair.body2->red_trans_verts[num+1];


		if(!checkForRepeat(p1, simplex))
			simplex.push_back(p1);

		num++;
	}

	sim = simplex;

	return true;
}




float RigidBodyManager :: EPA(CollisionPair pair, std::vector<glm::vec3> &simplex, glm::vec3 &coll_normal)
{
	glm::vec3 P;

	std::vector<std::vector<glm::vec3>> inside_faces;
	std::vector<std::vector<glm::vec3>> simplices;

	std::vector<glm::vec3> a_verts(3);
	std::vector<glm::vec3> b_verts(3);
	std::vector<glm::vec3> ab_verts(3);
	glm::vec3 a, b;

	int curr_simplex_num;

	simplices.push_back(simplex);

	while(true)
	{
		std::vector<glm::vec3> face;
		bool got_min = false;
		float min_dist = 0.0f;
		for(int i=0; i<simplices.size(); i++)
		{
			glm::vec3 p;
			std::vector<glm::vec3> f = getJohnsonsPlaneIgnoreFace(simplices[i], inside_faces, p);
			float dist = glm::length2(p);

			if((!got_min)||(dist < min_dist))
				if(f.size() == 3)
				{
					min_dist = dist;
					face = f;
					P = p;
					got_min = true;
				}
		}

		glm::vec3 norm;

		norm = glm::cross(face[1] - face[0], face[2] - face[0]);



		if(glm::dot(norm, face[0]) <= 0)
			norm = -norm;




		glm::vec3 V = diffSupportMap(pair, norm, a, b);


		glm::vec3 n_norm = glm::normalize(norm);




		float totalDist = glm::dot(V, n_norm);
		float simplexDist = glm::length(P);

		if(totalDist - simplexDist < 0.01f)
		{
			glm::vec3 p1, p2;
			glm::vec3 a = diffSupportMap(pair, -face[0], p1, p2);
			coll_normal = norm;
			return totalDist;
		}

		inside_faces.push_back(face);
		face.push_back(V);	
		simplices.push_back(face);

		cullSimplices(simplices, inside_faces);
	}

	return false;
}

glm::vec3 RigidBodyManager :: diffSupportMap(CollisionPair pair, glm::vec3 dir, glm::vec3 &v1, glm::vec3 &v2)
{
	return diffSupportMap(pair.body1->red_trans_verts, pair.body2->red_trans_verts,  dir, v1, v2);
}

glm::vec3 RigidBodyManager :: diffSupportMap(CollisionPair pair, glm::vec3 dir)
{
	return diffSupportMap(pair.body1->red_trans_verts, pair.body2->red_trans_verts, dir);
}

inline bool RigidBodyManager :: checkForVertex(glm::vec3 p, std::vector<glm::vec3> &simplex, size_t sim_size)
{
	glm::vec3 q;
	for(int i=0; i < sim_size; i++)
	{
		glm::vec3* sim = &simplex[i];
		if(p.x == sim->x && p.y ==  sim->y && p.z ==  sim->z)
			return true;
	}


	return false;
}

bool RigidBodyManager :: checkSubSimplex(std::vector<glm::vec3> &sub, std::vector<glm::vec3> &sim)
{
	size_t sim_size = sim.size();
	size_t sub_size = sub.size();
	for(int i=0; i<sub_size; i++)
		if(!checkForVertex(sub[i], sim, sim_size))
			return false;

	return true;
}


std::vector<glm::vec3> RigidBodyManager :: getJohnsonsPlaneIgnoreFace(std::vector<glm::vec3> &simplex, std::vector<std::vector<glm::vec3>> &faces, glm::vec3 &P)
{
	size_t faces_size = faces.size();
	bool got_min = false;
	float min_dist = 0.0f;
	std::vector<glm::vec3> vs;
	std::vector<glm::vec3>* curr_face;
	glm::vec3 a;
	glm::vec3 b;
	glm::vec3 c;
	size_t simplex_size = simplex.size();
	for(int i=0; i<simplex_size; i++)
		for(int j=i+1; j<simplex_size; j++)
		{
			for(int k=j+1; k<simplex_size; k++)
			{
				a = simplex[i];
				b = simplex[j];
				c = simplex[k];
				/*
				bool ignore = false;
				for(int l=0; l<faces.size(); l++)
				if((checkForVertex(a, faces[l]))&&(checkForVertex(b, faces[l]))&&(checkForVertex(c, faces[l])))
				{
				ignore = true;
				break;
				}




				if(ignore)
				continue;

				*/
				glm::vec3 ab = b-a;
				glm::vec3 ac = c-a;

				glm::vec3 norm = glm::cross(ab, ac);

				glm::vec3 n_norm = norm;
				if(norm != glm::vec3(0.0f))
					n_norm = glm::normalize(norm);

				float this_dist = glm::dot(n_norm, -a);

				if((abs(this_dist) < abs(min_dist))||(!got_min))
				{
					glm::vec3 p = -this_dist*n_norm;

					glm::vec3 vmin, vmax;
					vmin = vmax = a;

					if(b.x < vmin.x)
						vmin.x = b.x;
					if(b.y < vmin.y)
						vmin.y = b.y;
					if(b.z < vmin.z)
						vmin.z = b.z;

					if(b.x > vmax.x)
						vmax.x = b.x;
					if(b.y > vmax.y)
						vmax.y = b.y;
					if(b.z > vmax.z)
						vmax.z = b.z;


					if(c.x < vmin.x)
						vmin.x = c.x;
					if(c.y < vmin.y)
						vmin.y = c.y;
					if(c.z < vmin.z)
						vmin.z = c.z;

					if(c.x > vmax.x)
						vmax.x = c.x;
					if(c.y > vmax.y)
						vmax.y = c.y;
					if(c.z > vmax.z)
						vmax.z = c.z;


					if(((p.x <= vmax.x)&&(p.x >= vmin.x)&&(p.y <= vmax.y)&&(p.y >= vmin.y)&&(p.z <= vmax.z)&&(p.z >= vmin.z))||(!got_min))
					{
						bool ignore = false;
						for(int l=0; l<faces_size; l++)
						{

							curr_face = &faces[l];
							if((checkForVertex(a, *curr_face, 3))&&(checkForVertex(b, *curr_face, 3))&&(checkForVertex(c, *curr_face, 3)))
							{
								ignore = true;
								break;
							}
						}

						if(ignore)
							continue;

						got_min = true;
						vs.clear();
						vs.push_back(a);
						vs.push_back(b);
						vs.push_back(c);
						min_dist = this_dist;
						P = p;

					}
				}
			}
		}



		return vs;
}


void RigidBodyManager :: cullSimplices(std::vector<std::vector<glm::vec3>> &simplices, std::vector<std::vector<glm::vec3>> &faces)
{
	std::vector<int> times_ignored(faces.size());

	for(int i=0; i<faces.size(); i++)
		times_ignored[i] = 0;


	int num_simplices = simplices.size();
	for(int i=0; i<num_simplices; i++)
	{
		int num_faces = 0;

		for(int j=0; j<faces.size(); j++)
			if(checkSubSimplex(faces[j], simplices[i]))
			{
				num_faces++;
				times_ignored[j]++;

				if(num_faces >= 4)
					break;
			}

			if(num_faces >= 4)
			{
				simplices.erase(simplices.begin() + i);
				num_simplices--;
			}
	}

	for(int i=times_ignored.size()-1; i>=0; i--)
	{
		if(times_ignored[i] == 0)
			faces.erase(faces.begin() + i);
	}


}


std::vector<std::vector<glm::vec3>> RigidBodyManager :: splitTetrahedron(std::vector<glm::vec3> &tet)
{
	std::vector<std::vector<glm::vec3>> faces;

	for(int i=0; i<tet.size(); i++)
		for(int j=i+1; j<tet.size(); j++)
			for(int k=j+1; k<tet.size(); k++)
			{
				std::vector<glm::vec3> f(3);
				f[0] = tet[i];
				f[1] = tet[j];
				f[2] = tet[k];
				faces.push_back(f);
			}

			return faces;
}

int RigidBodyManager :: getJohnsonsPlane(std::vector<std::vector<glm::vec3>> &faces, glm::vec3 &P)
{
	bool got_min = false;
	float min_dist = 0.0f;
	int num = 0;


	for(int i=0; i<faces.size(); i++)
	{

		glm::vec3 a = faces[i][0];
		glm::vec3 b = faces[i][1];
		glm::vec3 c = faces[i][2];

		glm::vec3 ab = b-a;
		glm::vec3 ac = c-a;

		glm::vec3 norm = glm::cross(ab, ac);

		glm::vec3 n_norm = norm;
		if(norm != glm::vec3(0.0f))
			n_norm = glm::normalize(norm);

		float this_dist = glm::dot(n_norm, -a);

		if((abs(this_dist) < abs(min_dist))||(!got_min))
		{


			glm::vec3 p = -this_dist*n_norm;

			glm::vec3 vmin, vmax;
			vmin = vmax = a;

			if(b.x < vmin.x)
				vmin.x = b.x;
			if(b.y < vmin.y)
				vmin.y = b.y;
			if(b.z < vmin.z)
				vmin.z = b.z;

			if(b.x > vmax.x)
				vmax.x = b.x;
			if(b.y > vmax.y)
				vmax.y = b.y;
			if(b.z > vmax.z)
				vmax.z = b.z;


			if(c.x < vmin.x)
				vmin.x = c.x;
			if(c.y < vmin.y)
				vmin.y = c.y;
			if(c.z < vmin.z)
				vmin.z = c.z;

			if(c.x > vmax.x)
				vmax.x = c.x;
			if(c.y > vmax.y)
				vmax.y = c.y;
			if(c.z > vmax.z)
				vmax.z = c.z;


			if(((p.x <= vmax.x)&&(p.x >= vmin.x)&&(p.y <= vmax.y)&&(p.y >= vmin.y)&&(p.z <= vmax.z)&&(p.z >= vmin.z))||(!got_min))
			{
				got_min = true;
				num = i;
				min_dist = this_dist;
				P = p;
			}


		}

	}

	return num;
}


float RigidBodyManager :: optEPA(CollisionPair pair, std::vector<glm::vec3> simplex, glm::vec3 &coll_normal)
{
	glm::vec3 P;

	std::vector<std::vector<glm::vec3>> faces = splitTetrahedron(simplex);

	while(true)
	{
		int face_num = getJohnsonsPlane(faces, P);
		std::vector<glm::vec3> curr_face = faces[face_num];




		glm::vec3 norm = glm::cross(curr_face[1] - curr_face[0], curr_face[2] - curr_face[0]);



		if(glm::dot(norm, curr_face[0]) <= 0)
			norm = -norm;




		glm::vec3 V = diffSupportMap(pair, norm);


		glm::vec3 n_norm = glm::normalize(norm);




		float totalDist = glm::dot(V, n_norm);
		float simplexDist = glm::length(P);

		if(totalDist - simplexDist < 0.01f)
		{
			coll_normal = norm;
			return totalDist;
		}
		/*
		faces.erase(faces.begin()+face_num);

		for(int i=0; i<curr_face.size(); i++)
		for(int j=i+1; j<curr_face.size(); j++)
		{
		std::vector<glm::vec3> f(3);
		f[0] = curr_face[i];
		f[1] = curr_face[j];
		f[2] = V;

		faces.push_back(f);
		}

		*/
		expandPoly(faces, V);


	}

	return false;
}



void RigidBodyManager :: expandPoly(std::vector<std::vector<glm::vec3>> &faces, glm::vec3 spoint)
{
	std::vector<int> seen_face_nums = seenFaces(faces, spoint);
	std::vector<std::vector<glm::vec3>> edges;
	std::vector<int> repeated_edges_nums;

	for(int i=seen_face_nums.size()-1; i>=0; i--)
	{
		int curr_face_num = seen_face_nums[i];

		for(int j=0; j<3; j++)
			for(int k=j+1; k<3; k++)
			{
				std::vector<glm::vec3> edge(2);
				edge[0] = faces[curr_face_num][j];
				edge[1] = faces[curr_face_num][k];

				int repeated_edge_num;
				if(checkForEdge(edge, edges, repeated_edge_num))
				{
					if(!checkForIntInVec(repeated_edge_num, repeated_edges_nums))
						repeated_edges_nums.push_back(repeated_edge_num);
				}
				else
					edges.push_back(edge);			
			}


			faces.erase(faces.begin() + curr_face_num);
	}

	for(int i=0; i<edges.size(); i++)
	{
		if(!checkForIntInVec(i, repeated_edges_nums))
		{
			std::vector<glm::vec3> f(3);
			f[0] = edges[i][0];
			f[1] = edges[i][1];
			f[2] = spoint;

			faces.push_back(f);
		}
	}

}

bool RigidBodyManager :: checkForIntInVec(int num, std::vector<int> &vec)
{
	for(int i=0; i<vec.size(); i++)
		if(num == vec[i])
			return true;

	return false;
}

std::vector<int> RigidBodyManager :: seenFaces(std::vector<std::vector<glm::vec3>> &faces, glm::vec3 point)
{
	std::vector<int> seen_face_nums;

	for(int i=0; i<faces.size(); i++)
	{
		glm::vec3 norm = glm::cross(faces[i][1] - faces[i][0], faces[i][2] - faces[i][0]);

		if(glm::dot(norm, faces[i][0]) <= 0)
			norm = -norm;
		//		glm::vec3 n_norm = glm::normalize(norm);


		float dot_prod = glm::dot(point-faces[i][0], norm);

		if(dot_prod > 0.0f)
			seen_face_nums.push_back(i);
	}

	return seen_face_nums;
	/*
	std::vector<int> seen_face_nums;


	for(int i=0; i<faces.size(); i++)
	{
	glm::vec3 face_centre = (faces[i][0]+faces[i][1]+faces[i][2])/3.0f;
	float sq_dist = glm::distance2(point, face_centre);
	bool closest = true;

	for(int j=0; j<faces.size(); j++)
	if(i != j)
	{
	glm::vec3 hit_pos;
	//	if(checkRayTriangleIntersection(faces[j][0], faces[j][1], faces[j][2], point, face_centre, hit_pos))
	if(checkRayTri(faces[j], point, face_centre, hit_pos))
	{
	float this_sq_dist = glm::distance2(point, hit_pos);
	if(this_sq_dist < sq_dist)
	{
	closest = false;
	break;
	}
	}
	}

	if(closest)
	seen_face_nums.push_back(i);
	}

	return seen_face_nums;
	*/
}



bool RigidBodyManager :: checkRayTriangleIntersection(glm::vec3 TP1, glm::vec3 TP2, glm::vec3 TP3, glm::vec3 LP1, glm::vec3 LP2, glm::vec3 &HitPos)
{
	glm::vec3 Normal, IntersectPos;

	// Find Triangle Normal
	Normal = glm::cross( TP2 - TP1, TP3 - TP1 );
	glm::normalize(Normal); // not really needed

	// Find distance from LP1 and LP2 to the plane defined by the triangle
	float Dist1 = glm::dot(LP1-TP1, Normal);;
	float Dist2 = glm::dot(LP2-TP1, Normal);



	//	if ( (Dist1 * Dist2) >= 0.0f) 
	//		return false;	// line doesn't cross the triangle.
	if ( Dist1 == Dist2) 
		return false;	// line and plane are parallel



	// Find point on the line that intersects with the plane
	IntersectPos = LP1 + (LP2-LP1) * ( -Dist1/(Dist2-Dist1) );

	// Find if the interesection point lies inside the triangle by testing it against all edges
	glm::vec3 vTest;

	vTest = glm::cross( Normal, TP2-TP1 );
	if ( glm::dot( vTest, IntersectPos-TP1) <= 0.0f )
		return false;

	vTest = glm::cross( Normal, TP3-TP2 );
	if ( glm::dot( vTest, IntersectPos-TP2) <= 0.0f )
		return false;

	vTest = glm::cross( Normal, TP1-TP3 );
	if ( glm::dot( vTest, IntersectPos-TP3) <= 0.0f )
		return false;




	HitPos = IntersectPos;
	return true;
}




bool RigidBodyManager :: checkForEdge(std::vector<glm::vec3> new_edge, std::vector<std::vector<glm::vec3>> &edges, int &num)
{
	for(int i=0; i<edges.size(); i++)
	{
		if(new_edge[0] == edges[i][0])
			if(new_edge[1] == edges[i][1])
			{
				num = i;
				return true;
			}
			else 
				continue;

		if(new_edge[0] == edges[i][1])
			if(new_edge[1] == edges[i][0])
			{
				num = i;
				return true;
			}

	}
	return false;
}



bool RigidBodyManager :: checkRayTri(std::vector<glm::vec3> &tri, glm::vec3 source, glm::vec3 dest, glm::vec3 &hitPos)
{
	glm::vec3 e1 = tri[1] - tri[0];
	glm::vec3 e2 = tri[2] - tri[0];

	glm::vec3 ray = dest - source;

	glm::vec3 h = glm::cross(ray, e2);
	float a = glm::dot(e1, h);

	if(a == 0.0f)
		return false;

	float f = 1/a;

	glm::vec3 s = source - tri[0];
	float u = f * glm::dot(s, h);

	if((u<0.0f)||(u>1.0f))
		return false;

	glm::vec3 q = glm::cross(s, e1);
	float v = f * glm::dot(ray, q);

	if((v<0.0f)||(u+v > 1.0f))
		return false;

	float t = f * glm::dot(e2, q);

	hitPos = source + t*dest;
	return true;
}

bool RigidBodyManager :: checkForRepeat(glm::vec3 v, std::vector<glm::vec3> &verts)
{
	for(int i=0; i<verts.size(); i++)
		if(glm::distance2(v, verts[i]) < 0.000001)
			return true;

	return false;

}



glm::vec3 RigidBodyManager :: getBarycentricCoords(glm::vec3 p, std::vector<glm::vec3> &tri)
{


	glm::vec3 a, b, c;
	a = tri[0];
	b = tri[1];
	c = tri[2];

	glm::vec3 v0, v1, v2;
	v0 = b-a;
	v1 = c-a;
	v2 = p-a;

	float d00 = glm::dot(v0, v0);
	float d01 = glm::dot(v0, v1);
	float d11 = glm::dot(v1, v1);
	float d20 = glm::dot(v2, v0);
	float d21 = glm::dot(v2, v1);

	float denom = d00 * d11 - d01 * d01;

	float l1 = (d11 * d20 - d01 * d21) / denom;
    float l2 = (d00 * d21 - d01 * d20) / denom;
    float l3 = 1.0f - l1 - l2;

	return glm::vec3(l3, l1, l2);
}

float RigidBodyManager :: EPA_GJK(CollisionPair pair, glm::vec3 &coll_normal, glm::vec3 &pointA, glm::vec3 &pointB, std::vector<glm::vec3> &a_tri, std::vector<glm::vec3> &b_tri)
{
	std::vector<glm::vec3> simplex;
	std::vector<SupportPoint> sim;
	float dist = 0.0f;


	std::vector<glm::vec3> at, bt;

	if(epaGJK(pair, sim))
	{
		glm::vec3 p;
		dist = EPA(pair, sim, coll_normal, p);
		std::vector<glm::vec3> tri (3);
		tri[0] = sim[0].sc;
		tri[1] = sim[1].sc;
		tri[2] = sim[2].sc;

		at.push_back(sim[0].sa);
		at.push_back(sim[1].sa);
		at.push_back(sim[2].sa);
		bt.push_back(sim[0].sb);
		bt.push_back(sim[1].sb);
		bt.push_back(sim[2].sb);

		a_tri = at;
		b_tri = bt;

		glm::vec3 lambda = getBarycentricCoords(p, tri);

		pointA = lambda.x*sim[0].sa + lambda.y*sim[1].sa + lambda.z*sim[2].sa;
		pointB = lambda.x*sim[0].sb + lambda.y*sim[1].sb + lambda.z*sim[2].sb;


		float dist2 = glm::distance(pointA, pointB);

		
		ContactModel cm(pair, pointA, pointB, coll_normal);

		float impulse = cm.calcImpulse(coeffRestitution);
		glm::vec3 n_norm = glm::normalize(coll_normal);




		glm::vec3 force = impulse * n_norm;

		pair.body1->velocity += force / pair.body1->mass;
		pair.body2->velocity += -force / pair.body2->mass;
		
		glm::vec3 torque1 = glm::cross((pointA - pair.body1->centre_of_mass), force);
		glm::vec3 torque2 = glm::cross((pointB - pair.body2->centre_of_mass), -force);

		
		glm::mat3 moment_inertia1 = pair.body1->calcMomentInertia(pair.body1->rotation_mat);
		glm::mat3 inv_moment1 = glm::inverse(moment_inertia1);

		pair.body1->ang_velocity += inv_moment1 * torque1;



		glm::mat3 moment_inertia2 = pair.body2->calcMomentInertia(pair.body2->rotation_mat);
		glm::mat3 inv_moment2 = glm::inverse(moment_inertia2);

		pair.body2->ang_velocity += inv_moment2 * torque2;

	}



	return dist;
}




bool RigidBodyManager :: epaGJK(CollisionPair pair, std::vector<SupportPoint> &sim)
{
	std::vector<SupportPoint> simplex;


	 SupportPoint sp(pair.body1->red_trans_verts[0], pair.body2->red_trans_verts[0]);


	simplex.push_back(sp);

	bool intersection = false;
	glm::vec3 P;
	int num = 0;
	while(num < 100)
	{
		getJohnsons(simplex, simplex, P);


		if(P == glm::vec3(0.0f))
			break;



		glm::vec3 n_p = glm::normalize(-P);

		diffSupportMap(pair, -P, sp);
		glm::vec3 V = sp.sc;



		float v_dot = glm::dot(V, n_p);
		float p_dot = glm::dot(P, n_p);

		if(v_dot <= p_dot+0.05f)
			return false;


		simplex.push_back(sp);

		num++;
	}
	num = 0;
	int num2 = 0;
	int max_num = min(pair.body1->red_trans_verts.size(), pair.body2->red_trans_verts.size());
	while(simplex.size() < 4)
	{
		SupportPoint sp1(pair.body1->red_trans_verts[num], pair.body2->red_trans_verts[num2]);

		size_t simplex_size = simplex.size();
		bool repeat = false;
		for(int i=0; i<simplex_size; i++)
			if(sp1.sc == simplex[i].sc)
			{
				repeat = true;
				break;
			}

		if(!repeat)
			simplex.push_back(sp1);


		if(num != max_num-1)
			num++;
		else
		{
			num = 0;
			num2++;
		}
	}

	sim = simplex;

	return true;
}


void RigidBodyManager :: diffSupportMap(CollisionPair pair, glm::vec3 dir, SupportPoint &sp)
{
	glm::vec3 a, b, c;
	c = diffSupportMap(pair, dir, a, b);
	sp = SupportPoint(c, a, b);
}






void RigidBodyManager :: getJohnsons(std::vector<SupportPoint> &result_simplex, std::vector<SupportPoint> &simplex, glm::vec3 &P)
{
	std::vector<SupportPoint> vs;

	size_t simplex_size = simplex.size();

	for(int i=0; i<simplex_size; i++)
	{
		bool closest_point = true;
		glm::vec3 a = simplex[i].sc;
		glm::vec3 ax = -a;

		for(int j=0; j<simplex_size; j++)
			if(i != j)
			{
				glm::vec3 b = simplex[j].sc;


				glm::vec3 ab = b-a;

				float ax_dot_ab = glm::dot(ax, ab);
				if(ax_dot_ab > 0)
				{
					closest_point = false;
					break;
				}
			}

			if(closest_point)
			{
				P = a;
				vs.push_back(simplex[i]);
				result_simplex = vs;
				return;
			}
	}


	for(int i=0; i<simplex_size-1; i++)
		for(int j=i+1; j<simplex_size; j++)
		{
			bool closest_edge = true;

			glm::vec3 a = simplex[i].sc;
			glm::vec3 b = simplex[j].sc;

			if(a == b)
			{
				closest_edge = false;
				break;
			}

			glm::vec3 ax = -a;
			glm::vec3 bx = -b;

			glm::vec3 ab = b-a;
			glm::vec3 ba = a-b;

			float ax_dot_ab = glm::dot(ax, ab);
			if(ax_dot_ab < 0)
			{
				closest_edge = false;
				break;
			}

			float bx_dot_ba = glm::dot(bx, ba);
			if(bx_dot_ba < 0)
			{
				closest_edge = false;
				break;
			}


			for(int k=0; k<simplex_size; k++)
				if((k != i)&&(k != j))
				{
					glm::vec3 c = simplex[k].sc;

					glm::vec3 bc = c-b;

					glm::vec3 cross = glm::cross(bc, ba);
					cross = glm::cross(cross, ba);

					float cross_dot_bx = glm::dot(cross, bx);
					if(cross_dot_bx < 0)
					{
						closest_edge = false;
						break;
					}
				}


				if(closest_edge)
				{

					glm::vec3 n_ab = glm::normalize(ab);
					float ax_dot_nab = glm::dot(ax, n_ab);

					P = a + ax_dot_nab*n_ab;



					vs.push_back(simplex[i]);
					vs.push_back(simplex[j]);
					result_simplex = vs;
					return;
				}
		}

		getJohnsonsPlane(vs, simplex, P);

		result_simplex = vs;
		return;
}






void RigidBodyManager :: getJohnsonsPlane(std::vector<SupportPoint> &plane, std::vector<SupportPoint> &simplex, glm::vec3 &P)
{
	bool got_min = false;
	float min_dist = 0.0f;
	std::vector<SupportPoint> vs;

	size_t simplex_size = simplex.size();

	for(int i=0; i<simplex_size-2; i++)
		for(int j=i+1; j<simplex_size-1; j++)
			for(int k=j+1; k<simplex_size; k++)
			{

				glm::vec3 a = simplex[i].sc;
				glm::vec3 b = simplex[j].sc;
				glm::vec3 c = simplex[k].sc;

				glm::vec3 ab = b-a;
				glm::vec3 ac = c-a;

				glm::vec3 norm = glm::cross(ab, ac);

				glm::vec3 n_norm = norm;
				if(norm != glm::vec3(0.0f))
					n_norm = glm::normalize(norm);

				float this_dist = glm::dot(n_norm, -a);

				if((abs(this_dist) < abs(min_dist))||(!got_min))
				{


					glm::vec3 p = -this_dist*n_norm;

					glm::vec3 vmin, vmax;
					vmin = vmax = a;

					if(b.x < vmin.x)
						vmin.x = b.x;
					if(b.y < vmin.y)
						vmin.y = b.y;
					if(b.z < vmin.z)
						vmin.z = b.z;

					if(b.x > vmax.x)
						vmax.x = b.x;
					if(b.y > vmax.y)
						vmax.y = b.y;
					if(b.z > vmax.z)
						vmax.z = b.z;


					if(c.x < vmin.x)
						vmin.x = c.x;
					if(c.y < vmin.y)
						vmin.y = c.y;
					if(c.z < vmin.z)
						vmin.z = c.z;

					if(c.x > vmax.x)
						vmax.x = c.x;
					if(c.y > vmax.y)
						vmax.y = c.y;
					if(c.z > vmax.z)
						vmax.z = c.z;


					if(((p.x <= vmax.x)&&(p.x >= vmin.x)&&(p.y <= vmax.y)&&(p.y >= vmin.y)&&(p.z <= vmax.z)&&(p.z >= vmin.z))||(!got_min))
					{
						got_min = true;
						vs.clear();
						vs.push_back(simplex[i]);
						vs.push_back(simplex[j]);
						vs.push_back(simplex[k]);
						min_dist = this_dist;
						P = p;
					}
				}
			}


	plane = vs;

}





float RigidBodyManager :: EPA(CollisionPair pair, std::vector<SupportPoint> &simplex, glm::vec3 &coll_normal, glm::vec3 &coll_point)
{
	glm::vec3 P;

	std::vector<std::vector<SupportPoint>> inside_faces;
	std::vector<std::vector<SupportPoint>> simplices;

	std::vector<SupportPoint> f;
	std::vector<SupportPoint> face;
	
	SupportPoint sp;


	int curr_simplex_num;

	simplices.push_back(simplex);

	while(true)
	{
		
		bool got_min = false;
		float min_dist = 0.0f;
		for(int i=0; i<simplices.size(); i++)
		{
			glm::vec3 p;
			getJohnsonsPlaneIgnoreFace(f, simplices[i], inside_faces, p);
			float dist = glm::length2(p);

			if((!got_min)||(dist < min_dist))
				if(f.size() == 3)
				{
					min_dist = dist;
					face = f;
					P = p;
					got_min = true;
				}
		}

		glm::vec3 norm;

		norm = glm::cross(face[1].sc - face[0].sc, face[2].sc - face[0].sc);



		if(glm::dot(norm, face[0].sc) <= 0)
			norm = -norm;



		diffSupportMap(pair, norm, sp);
		glm::vec3 V = sp.sc;


		glm::vec3 n_norm = glm::normalize(norm);




		float totalDist = glm::dot(V, n_norm);
		float simplexDist = glm::length(P);

		if(totalDist - simplexDist < 0.01f)
		{
			simplex = face;
			coll_point = P;
			coll_normal = norm;
			return simplexDist;
		}

		inside_faces.push_back(face);
		face.push_back(sp);	
		simplices.push_back(face);

		cullSimplices(simplices, inside_faces);
	}

	return false;
}







void RigidBodyManager :: cullSimplices(std::vector<std::vector<SupportPoint>> &simplices, std::vector<std::vector<SupportPoint>> &faces)
{
	std::vector<int> times_ignored(faces.size());
	size_t faces_size = faces.size();

	for(int i=0; i<faces_size; i++)
		times_ignored[i] = 0;


	int num_simplices = simplices.size();
	for(int i=0; i<num_simplices; i++)
	{
		int num_faces = 0;

		for(int j=0; j<faces_size; j++)
			if(checkSubSimplex(faces[j], simplices[i]))
			{
				num_faces++;
				times_ignored[j]++;

				if(num_faces >= 4)
					break;
			}

			if(num_faces >= 4)
			{
				simplices.erase(simplices.begin() + i);
				num_simplices--;
			}
	}

	for(int i=times_ignored.size()-1; i>=0; i--)
	{
		if(times_ignored[i] == 0)
			faces.erase(faces.begin() + i);
	}


}




void RigidBodyManager :: getJohnsonsPlaneIgnoreFace(std::vector<SupportPoint> &result, std::vector<SupportPoint> &simplex, std::vector<std::vector<SupportPoint>> &faces, glm::vec3 &P)
{
	size_t faces_size = faces.size();
	bool got_min = false;
	float min_dist = 0.0f;
	std::vector<SupportPoint> vs;
	std::vector<SupportPoint>* curr_face;
	glm::vec3 a;
	glm::vec3 b;
	glm::vec3 c;
	size_t simplex_size = simplex.size();
	for(int i=0; i<simplex_size; i++)
		for(int j=i+1; j<simplex_size; j++)
		{
			for(int k=j+1; k<simplex_size; k++)
			{
				a = simplex[i].sc;
				b = simplex[j].sc;
				c = simplex[k].sc;
				
				glm::vec3 ab = b-a;
				glm::vec3 ac = c-a;

				glm::vec3 norm = glm::cross(ab, ac);

				glm::vec3 n_norm = norm;
				if(norm != glm::vec3(0.0f))
					n_norm = glm::normalize(norm);

				float this_dist = glm::dot(n_norm, -a);

				if((abs(this_dist) < abs(min_dist))||(!got_min))
				{
					glm::vec3 p = -this_dist*n_norm;

					glm::vec3 vmin, vmax;
					vmin = vmax = a;

					if(b.x < vmin.x)
						vmin.x = b.x;
					if(b.y < vmin.y)
						vmin.y = b.y;
					if(b.z < vmin.z)
						vmin.z = b.z;

					if(b.x > vmax.x)
						vmax.x = b.x;
					if(b.y > vmax.y)
						vmax.y = b.y;
					if(b.z > vmax.z)
						vmax.z = b.z;


					if(c.x < vmin.x)
						vmin.x = c.x;
					if(c.y < vmin.y)
						vmin.y = c.y;
					if(c.z < vmin.z)
						vmin.z = c.z;

					if(c.x > vmax.x)
						vmax.x = c.x;
					if(c.y > vmax.y)
						vmax.y = c.y;
					if(c.z > vmax.z)
						vmax.z = c.z;


					if(((p.x <= vmax.x)&&(p.x >= vmin.x)&&(p.y <= vmax.y)&&(p.y >= vmin.y)&&(p.z <= vmax.z)&&(p.z >= vmin.z))||(!got_min))
					{
						bool ignore = false;
						for(int l=0; l<faces_size; l++)
						{

							curr_face = &faces[l];
							if((checkForVertex(a, *curr_face, 3))&&(checkForVertex(b, *curr_face, 3))&&(checkForVertex(c, *curr_face, 3)))
							{
								ignore = true;
								break;
							}
						}

						if(ignore)
							continue;

						got_min = true;
						vs.clear();
						vs.push_back(simplex[i]);
						vs.push_back(simplex[j]);
						vs.push_back(simplex[k]);
						min_dist = this_dist;
						P = p;

					}
				}
			}
		}


		result = vs;
}




inline bool RigidBodyManager :: checkForVertex(glm::vec3 p, std::vector<SupportPoint> &simplex, size_t sim_size)
{
	glm::vec3 q;
	for(int i=0; i < sim_size; i++)
	{
		glm::vec3* sim = &simplex[i].sc;
		if(p.x == sim->x && p.y ==  sim->y && p.z ==  sim->z)
			return true;
	}


	return false;
}



bool RigidBodyManager :: checkSubSimplex(std::vector<SupportPoint> &sub, std::vector<SupportPoint> &sim)
{
	size_t sim_size = sim.size();
	size_t sub_size = sub.size();



	for(int i=0; i<sub_size; i++)
		if(!checkForVertex(sub[i].sc, sim, sim_size))
			return false;

	return true;
}

bool RigidBodyManager :: EPA_GJK(CollisionPair pair)
{
	glm::vec3 cn, pa, pb;
	std::vector<glm::vec3> at, bt;
	float dist = EPA_GJK(pair, cn, pa, pb, at, bt);

	if(dist != 0.0f)
		return true;

	return false;
}


void RigidBodyManager :: adjustDragCoeff()
{
	for(int i=0; i<num; i++)
		bodies[i].drag_coeff = drag_coeff;
}