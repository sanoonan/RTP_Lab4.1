#include "AABB.h"

using namespace std;

AABB :: AABB()
{
	centre = glm::vec3(0.0f);
	height = width = 0;
}

AABB :: AABB(std::vector<glm::vec3> v)
{ 
	AABB aabb;
	aabb.update(v);
	*this = aabb;
}

void AABB :: update(std::vector<glm::vec3> tv)
{
	vmin = vmax = tv[0];
	int num = tv.size();

	for(int i=1; i<num; i++)
	{
		if(tv[i].x < vmin.x)
			vmin.x = tv[i].x;
		if(tv[i].y < vmin.y)
			vmin.y = tv[i].y;
		if(tv[i].z < vmin.z)
			vmin.z = tv[i].z;

		if(tv[i].x > vmax.x)
			vmax.x = tv[i].x;
		if(tv[i].y > vmax.y)
			vmax.y = tv[i].y;
		if(tv[i].z > vmax.z)
			vmax.z = tv[i].z;
	}
	width = vmax.x - vmin.x;
	height = vmax.y - vmin.y;
	depth = vmax.z - vmin.z;

	centre = (vmin+vmax)*0.5f;
}



void AABB :: draw(GLuint spID, glm::vec3 colour)
{
	glm::mat4 s_mat = glm::scale(glm::mat4(), glm::vec3(width, height, depth));
	glm::mat4 t_mat = glm::translate(glm::mat4(), centre);
	glm::mat4 b_mat = t_mat * s_mat;


	int model_matrix_location = glGetUniformLocation (spID, "model");
	glUniformMatrix4fv(model_matrix_location, 1, GL_FALSE, glm::value_ptr(b_mat));


	int colour_location = glGetUniformLocation (spID, "colour");
	glUniform3fv(colour_location, 1, glm::value_ptr(colour));

	glutWireCube(1);
}


bool AABB :: checkCollision(AABB other)
{
	float xdist = abs(centre.x - other.centre.x);
	float xdiff = width/2 + other.width/2;
	if(xdist > xdiff)
		return false;

	float ydist = abs(centre.y - other.centre.y);
	float ydiff = height/2 + other.height/2;
	if(ydist > ydiff)
		return false;

	float zdist = abs(centre.z - other.centre.z);
	float zdiff = depth/2 + other.depth/2;
	if(zdist > zdiff)
		return false;

	return true;
}