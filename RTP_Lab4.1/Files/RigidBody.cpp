#include "RigidBody.h"

using namespace std;

RigidBody :: RigidBody()
{
	position = orientation = velocity = ang_velocity = centre_of_mass = glm::vec3(0.0f);
	translation_mat = glm::mat4();
	inertial_tensor = rotation_mat = glm::mat3();
	mass = 0;
	drag_coeff = 0;

	no_hit_col = glm::vec3(0.0f, 1.0f, 0.0f);
	hit_col = glm::vec3(1.0f, 0.0f, 0.0f);
	collision = false;

	colour = glm::vec3(0.7f);

	scale = glm::vec3(1.0f);

	id = 0;
}

RigidBody :: RigidBody(Mesh _mesh)
{
	RigidBody body;
	body.mesh = _mesh;
	body.mass = 1;
	body.drag_coeff = 1;


	*this = body;
}

RigidBody :: RigidBody(const char* filename)
{
	RigidBody body;
	
	Mesh mesh(filename);

	body.mesh = mesh;
	body.mass = 1;
	body.drag_coeff = 1;


	*this = body;
}

void RigidBody :: load_mesh()
{
	mesh.load_mesh(vertices, reduced_verts);
	
	transformed_vertices.resize(vertices.size());

	for(int i=0; i<vertices.size(); i++)
		transformed_vertices[i] = vertices[i];

	red_trans_verts.resize(reduced_verts.size());
	for(int i=0; i<reduced_verts.size(); i++)
		red_trans_verts[i] = reduced_verts[i];

	original_com = updateCOM(vertices);
	inertial_tensor = calcInertialTensorBox();

	b_sphere = BoundingSphere(vertices);
	aabb = AABB(vertices);
}

void RigidBody :: draw(GLuint spID)
{	
	mesh.draw(spID, model_mat, colour);
}

void RigidBody :: updateBB()
{
	if(collision)
		current_col = hit_col;
	else
		current_col = no_hit_col;

	b_sphere.update(model_mat);
	aabb.update(transformed_vertices);
}


void RigidBody :: drawBSphere(GLuint spID)
{	
	b_sphere.draw(spID, current_col);
}

void RigidBody :: drawAABB(GLuint spID)
{
	aabb.draw(spID, current_col);
}



void RigidBody :: addTBar(TwBar *bar)
{
	char char_id = (char)(((int)'0')+id);

	TwAddVarRW(bar, char_id+"Position", TW_TYPE_DIR3F, &position, " label = 'Position' ");
//	TwAddVarRO(bar, char_id+"Orientation", TW_TYPE_DIR3F, &orientation, " label = 'Orientation' ");
//	TwAddVarRW(bar, char_id+"Velocity", TW_TYPE_DIR3F, &velocity, " label = 'Velocity' ");
//	TwAddVarRW(bar, char_id+"Angular Velocity", TW_TYPE_DIR3F, &ang_velocity, " label = 'Angular Velocity' ");
//	TwAddVarRW(bar, char_id+"Drag Coefficient", TW_TYPE_FLOAT, &drag_coeff, " label = 'Drag Coefficient' ");
//	TwAddVarRW(bar, char_id+"Drag Coefficient", TW_TYPE_FLOAT, &drag_coeff, " label = 'Drag Coefficient' ");
}

void RigidBody :: update(float dt)
{
	updateDrag();
	updateTranslation(dt);
	updateRotation(dt);
	cullInstances(force_instances, dt);

	glm::mat4 scale_mat = glm::scale(glm::mat4(), scale);

	model_mat = translation_mat * glm::mat4(rotation_mat) * scale_mat;
	centre_of_mass = glm::vec3(model_mat * glm::vec4(original_com, 1.0f));

	transformVertices();

	updateBB();
}



glm::vec3 RigidBody :: calcDrag(glm::vec3 v)
{
	glm::vec3 d = -drag_coeff * v;
	return d;
}

void RigidBody :: updateDrag()
{
	drag.force = calcDrag(velocity);
	drag.torque = calcDrag(ang_velocity);
}


void RigidBody :: updateTranslation(float dt)
{
	glm::vec3 total_force = calcTotalForceWithDrag(force_instances);


	velocity += total_force * dt / mass;


	position += dt * velocity;

	translation_mat = glm::translate(glm::mat4(), position);
}

void RigidBody :: updateRotation(float dt)
{
	glm::vec3 total_torque = calcTotalTorqueWithDrag(force_instances);

	glm::mat3 moment_inertia = calcMomentInertia(rotation_mat);
	glm::mat3 inv_moment = glm::inverse(moment_inertia);

	ang_velocity += (inv_moment * total_torque * dt) / mass;

	glm::mat3 ang_vel_mat = makeAngVelMat(ang_velocity);

	rotation_mat += ang_vel_mat * rotation_mat * dt;	

	rotation_mat = glm::orthonormalize(rotation_mat);

	orientation = getOrientationFromRotMat(rotation_mat);
}


void RigidBody :: updateTranslationRK4(float dt)
{
	std::vector<force_instance> forces;
	forces = force_instances;

	glm::vec3 other_forces, total_force;


	glm::vec3 x1, x2, x3, x4, v1, v2, v3, v4, a1, a2, a3, a4, x, v, xf, vf;

	x = position;
	v = velocity;

	x1 = x;
	v1 = v;
	other_forces = calcTotalForce(forces);
	total_force = other_forces + calcDrag(v1);
	a1 = total_force/mass;
	cullInstances(forces, 0);

	x2 += 0.5f * v1 * dt;
	v2 += 0.5f * a1 * dt;
	other_forces = calcTotalForce(forces);
	total_force = other_forces + calcDrag(v2);
	a2 = total_force/mass;

	x3 += 0.5f * v2 * dt;
	v3 += 0.5f * a2 * dt;
	other_forces = calcTotalForce(forces);
	total_force = other_forces + calcDrag(v3);
	a3 = total_force/mass;
	cullInstances(forces, dt/2);

	x4 += v3 * dt;
	v4 += a3 * dt;
	other_forces = calcTotalForce(forces);
	total_force = other_forces + calcDrag(v4);
	a4 = total_force/mass;
	
	xf = x + (dt/6)*(v1 + 2.0f * v2 + 2.0f * v3 + v4);
	vf = v + (dt/6)*(a1 + 2.0f * a2 + 2.0f * a3 + a4);
	
	velocity = vf;

	position = xf;

	translation_mat = glm::translate(glm::mat4(), position);

}



void RigidBody :: updateRotationRK4(float dt)
{

	std::vector<force_instance> torques;
	torques = force_instances;

	glm::vec3 other_torques, total_torque;

	glm::mat3 moment_inertia, inv_moment;



	glm::vec3 v1, v2, v3, v4, a1, a2, a3, a4, v, vf;
	glm::mat3 r1, r2, r3, r4, r, rf, av1, av2, av3, av4, av, avf;


	v = ang_velocity;
	r = rotation_mat;
	av = makeAngVelMat(v);


	v1 = v;
	r1 = r;
	r1 = glm::orthonormalize(r1);
	av1 = av;
	other_torques = calcTotalForce(torques);
	total_torque = other_torques + calcDrag(v1);
	moment_inertia = calcMomentInertia(r1);
	inv_moment = glm::inverse(moment_inertia);
	a1 = (inv_moment * total_torque) / mass;
	cullInstances(torques, 0);

	r2 += 0.5f * av1 * r1 * dt;	
	r2 = glm::orthonormalize(r2);
	v2 += 0.5f * a1 * dt;
	av2 = makeAngVelMat(v2);
	other_torques = calcTotalForce(torques);
	total_torque = other_torques + calcDrag(v2);
	moment_inertia = calcMomentInertia(r2);
	inv_moment = glm::inverse(moment_inertia);
	a2 = (inv_moment * total_torque) / mass;


	r3 += 0.5f * av2 * r2 * dt;	
	r3 = glm::orthonormalize(r3);
	v3 += 0.5f * a2 * dt;
	av3 = makeAngVelMat(v3);
	other_torques = calcTotalForce(torques);
	total_torque = other_torques + calcDrag(v3);
	moment_inertia = calcMomentInertia(r3);
	inv_moment = glm::inverse(moment_inertia);
	a3 = (inv_moment * total_torque) / mass;
	cullInstances(torques, dt/2);

	r4 += av3 * r2 * dt;
	r4 = glm::orthonormalize(r4);
	v4 += a3 * dt;
	av4 = makeAngVelMat(v4);
	other_torques = calcTotalForce(torques);
	total_torque = other_torques + calcDrag(v4);
	moment_inertia = calcMomentInertia(r4);
	inv_moment = glm::inverse(moment_inertia);
	a4 = (inv_moment * total_torque) / mass;


	rf = r + (dt/6)*(av1 * r1 + 2.0f * av2 * r2 + 2.0f * av3 * r3 + av4 * r4);
	rf = glm::orthonormalize(rf);
	vf = v + (dt/6)*(a1 + 2.0f * a2 + 2.0f * a3 + a4);

	rotation_mat = rf;
	ang_velocity = vf;
	orientation = getOrientationFromRotMat(rotation_mat);
}




glm::mat3 RigidBody :: makeAngVelMat(glm::vec3 av_vec)
{
	glm::mat3 av;

	av[0][1] = -av_vec.z;
	av[0][2] = av_vec.y;

	av[1][0] = av_vec.z;
	av[1][2] = -av_vec.x;

	av[2][0] = -av_vec.y;
	av[2][1] = av_vec.x;

	return av;
}




glm::vec3 RigidBody :: getOrientationFromRotMat(glm::mat3 mat)
{
	float x, y, z;
	glm::vec3 vec;

	x = atan2(mat[2][1], mat[2][2]);
	y = atan2(-mat[2][0], sqrt(mat[2][1]*mat[2][1] + mat[2][2]*mat[2][2]));
	z = atan2(mat[1][0], mat[0][0]);

	x = glm::degrees(x);
	y = glm::degrees(y);
	z = glm::degrees(z);

	while(x < 0)
		x += 360;

	while(y < 0)
		y += 360;

	while(z < 0)
		z += 360;

	vec = glm::vec3(x, y, z);
	return vec;

}

glm::mat3 RigidBody :: calcInertialTensorBox()
{
	float width, height, depth;
	float min_x, min_y, min_z, max_x, max_y, max_z;

	min_x = max_x = vertices[0].x;
	min_y = max_y = vertices[0].y;
	min_z = max_z = vertices[0].z;

	for(int i=1; i<vertices.size(); i++)
	{
		if(vertices[i].x < min_x)
			min_x = vertices[i].x;
		if(vertices[i].x > max_x)
			max_x = vertices[i].x;

		if(vertices[i].y < min_y)
			min_y = vertices[i].y;
		if(vertices[i].y > max_y)
			max_y = vertices[i].y;

		if(vertices[i].z < min_z)
			min_z = vertices[i].z;
		if(vertices[i].z > max_z)
			max_z = vertices[i].z;
	}

	width = max_x - min_x;
	height = max_y - min_y;
	depth = max_z - min_z;

	glm::mat3 it;
	float a, b, c;

	a = (mass*(height*height + depth*depth))/12;
	b = (mass*(width*width + depth*depth))/12;
	c = (mass*(width*width + height*height))/12;

	it[0][0] = a;
	it[1][1] = b;
	it[2][2] = c;

	return it;
}

glm::mat3 RigidBody :: calcMomentInertia(glm::mat3 rot_mat)
{
	glm::mat3 rt = glm::transpose(rot_mat);
	glm::mat3 mi = rot_mat * inertial_tensor * rt;

	return mi;
}


void RigidBody :: affectedByForce(Effector effector)
{
	force_instance fi;
	fi.force = effector.force_mag * effector.force_dir;
	fi.time_left = effector.time;

	fi.torque = glm::cross((centre_of_mass - effector.position), fi.force);

	

	int fi_list_size = force_instances.size();
	force_instances.resize(fi_list_size + 1);
	force_instances[fi_list_size] = fi;
}


glm::vec3 RigidBody :: calcTotalForce(std::vector<force_instance> fi)
{
	glm::vec3 t = glm::vec3(0.0f);

	int size = fi.size();

	for(int i=0; i<size; i++)
		t += fi[i].force;


	return t;
}


glm::vec3 RigidBody :: calcTotalTorque(std::vector<force_instance> fi)
{
	glm::vec3 t = glm::vec3(0.0f);

	int size = fi.size();

	for(int i=0; i<size; i++)
		t += fi[i].torque;



	return t;
}

glm::vec3 RigidBody :: calcTotalForceWithDrag(std::vector<force_instance> fi)
{
	glm::vec3 t = calcTotalForce(fi);

	t+= drag.force;

	return t;
}


glm::vec3 RigidBody :: calcTotalTorqueWithDrag(std::vector<force_instance> fi)
{
	glm::vec3 t = calcTotalTorque(fi);

	t+= drag.torque;

	return t;
}



void RigidBody :: cullInstances(std::vector<force_instance> &fi, float dt)
{
	int size = fi.size();

	for(int i=0; i<size; i++)
	{
		fi[i].time_left -= dt;
		if(fi[i].time_left <= 0)
		{
			removeInstance(fi, i);
			size--;
		}
	}
}


void RigidBody :: removeInstance(std::vector<force_instance> &fi, int num)
{
	int size = fi.size();

	if(num >= size)
		return;

	for(int i=num; i<size-1; i++)
		fi[i] = fi[i+1];

	fi.resize(size-1);
}

void RigidBody :: transformVertices()
{

	for(int i=0; i<vertices.size(); i++)
		transformed_vertices[i] = glm::vec3(model_mat *  glm::vec4(vertices[i], 1.0));

	for(int i=0; i<reduced_verts.size(); i++)
		red_trans_verts[i] = glm::vec3(model_mat * glm::vec4(reduced_verts[i], 1.0));

}

glm::vec3 RigidBody :: updateCOM(std::vector<glm::vec3> v)
{
	/*
	float totx = 0;
	float toty = 0;
	float totz = 0;
	glm::vec3 c;
	int size = v.size();
	for(int i=0; i<size; i++)
	{
		totx += v[i].x;
		toty += v[i].y;
		totz += v[i].z;
	}
	totx /= size;
	toty /= size;
	totz /= size;

	c = glm::vec3(totx, toty, totz);

	return c;
	*/
	float width, height, depth;
	float min_x, min_y, min_z, max_x, max_y, max_z;

	min_x = max_x = v[0].x;
	min_y = max_y = v[0].y;
	min_z = max_z = v[0].z;

	for(int i=1; i<v.size(); i++)
	{
		if(v[i].x < min_x)
			min_x = v[i].x;
		if(v[i].x > max_x)
			max_x = v[i].x;

		if(v[i].y < min_y)
			min_y = v[i].y;
		if(v[i].y > max_y)
			max_y = v[i].y;

		if(v[i].z < min_z)
			min_z = v[i].z;
		if(v[i].z > max_z)
			max_z = v[i].z;
	}

	glm::vec3 min(min_x, min_y, min_z);
	glm::vec3 max(max_x, max_y, max_z);

	glm::vec3 mid = (min + max) * (0.5f);
	
	return mid;

}


void RigidBody :: reset()
{
	position = orientation = velocity = ang_velocity = centre_of_mass = glm::vec3(0.0f);
	translation_mat  = glm::mat4();
	inertial_tensor = rotation_mat = glm::mat3();
	mass = 1;
	drag_coeff = 1;

	while(force_instances.size() > 0)
		removeInstance(force_instances, 0);
}



bool RigidBody :: checkRayHit(glm::vec3 ray_origin, glm::vec3 p1, glm::vec3 p2, glm::vec3 &hitPos)
{
	float min_hit_dist;
	glm::vec3 min_hit_pos;
	bool hit = false;

	for(int i=0; i < transformed_vertices.size()-2; i+=3)
	{
		glm::vec3 hit_position;
		if(checkRayTriangleIntersection(transformed_vertices[i], transformed_vertices[i+1], transformed_vertices[i+2], p1, p2, hit_position))
		{
			float hit_dist = glm::distance(ray_origin, hit_position);

			if(!hit)
			{
				hit = true;
				min_hit_pos = hit_position;
				min_hit_dist = hit_dist;
			}
			else
			{
				if(hit_dist < min_hit_dist)
				{
					min_hit_dist = hit_dist;
					min_hit_pos = hit_position;
				}
			}
		}
	}

	if(hit)
		hitPos = min_hit_pos;

	return hit;
}


bool RigidBody :: checkRayTriangleIntersection(glm::vec3 TP1, glm::vec3 TP2, glm::vec3 TP3, glm::vec3 LP1, glm::vec3 LP2, glm::vec3 &HitPos)
{
	glm::vec3 Normal, IntersectPos;

	// Find Triangle Normal
	Normal = glm::cross( TP2 - TP1, TP3 - TP1 );
	glm::normalize(Normal); // not really needed

	// Find distance from LP1 and LP2 to the plane defined by the triangle
	float Dist1 = glm::dot(LP1-TP1, Normal);;
	float Dist2 = glm::dot(LP2-TP1, Normal);



	if ( (Dist1 * Dist2) >= 0.0f) 
		return false;	// line doesn't cross the triangle.
	if ( Dist1 == Dist2) 
		return false;	// line and plane are parallel

	// Find point on the line that intersects with the plane
	IntersectPos = LP1 + (LP2-LP1) * ( -Dist1/(Dist2-Dist1) );

	// Find if the interesection point lies inside the triangle by testing it against all edges
	glm::vec3 vTest;
	
	vTest = glm::cross( Normal, TP2-TP1 );
	if ( glm::dot( vTest, IntersectPos-TP1) < 0.0f )
		return false;
	
	vTest = glm::cross( Normal, TP3-TP2 );
	if ( glm::dot( vTest, IntersectPos-TP2) < 0.0f )
		return false;
	
	vTest = glm::cross( Normal, TP1-TP3 );
	if ( glm::dot( vTest, IntersectPos-TP1) < 0.0f )
		return false;

	HitPos = IntersectPos;
	return true;
}

