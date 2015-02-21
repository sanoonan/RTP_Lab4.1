#include "BoundingSphere.h"

using namespace std;

BoundingSphere :: BoundingSphere()
{
	centre = glm::vec3(0.0f);
	radius = 0;
}

BoundingSphere :: BoundingSphere(glm::vec3 _centre, float _radius)
{
	centre = _centre;
	radius = _radius;
}




BoundingSphere :: BoundingSphere(std::vector<glm::vec3> v)
{
/*
	glm::vec3 vmin, vmax;

	vmin = vmax = v[0];

	for(int i=1; i<v.size(); i++)
	{
		if(v[i].x < vmin.x)
			vmin.x = v[i].x;
		if(v[i].y < vmin.y)
			vmin.y = v[i].y;
		if(v[i].z < vmin.z)
			vmin.z = v[i].z;

		if(v[i].x > vmax.x)
			vmax.x = v[i].x;
		if(v[i].y > vmax.y)
			vmax.y = v[i].y;
		if(v[i].z > vmax.z)
			vmax.z = v[i].z;
	}
	glm::vec3 mid = (vmax+vmin)*(0.5f);
	glm::vec3 diff = vmax-vmin;
	float distance = glm::length(diff);

	centre = mid;
	radius = distance/2;

	_min = vmin;
	_max = vmax;

	_v = v;
	*/
	/*
	float max_dist = 0;
	glm::vec3 mid;

	int num = v.size();

	for(int i=0; i<num; i++)
		for(int j=0; j<num; j++)
		{
			float distance = glm::length(v[i]-v[j]);
			if(distance > max_dist)
			{
				max_dist = distance;
				mid = (v[i] + v[j]) * (0.5f);
				_min = v[i];
				_max = v[j];
			}
		}

	centre = mid;
	radius = max_dist/2;
	*/
	/*	
	float max_dist = 0;
	glm::vec3 mid;

	int num = v.size();

	for(int i=0; i<num; i++)
		for(int j=0; j<num; j++)
		{
			float distance = glm::length(v[i]-v[j]);
			if(distance > max_dist)
			{
				max_dist = distance;
				mid = (v[i] + v[j]) * (0.5f);
			}
		}

	centre = mid;
	radius = max_dist/2;
	*/
	


	//RITTERS BOUNDING SPHERE
	glm::vec3 vmin, vmax;

	vmin = vmax = v[0];
	int num = v.size();

	for(int i=1; i<num; i++)
	{
		if(v[i].x < vmin.x)
			vmin.x = v[i].x;
		if(v[i].y < vmin.y)
			vmin.y = v[i].y;
		if(v[i].z < vmin.z)
			vmin.z = v[i].z;

		if(v[i].x > vmax.x)
			vmax.x = v[i].x;
		if(v[i].y > vmax.y)
			vmax.y = v[i].y;
		if(v[i].z > vmax.z)
			vmax.z = v[i].z;
	}
	float xdiff = vmax.x - vmin.x;
	float ydiff = vmax.y - vmin.y;
	float zdiff = vmax.z - vmin.y;

	glm::vec3 mid = (vmax+vmin)*(0.5f);

	float max_dist = max(xdiff, max(ydiff, zdiff));


	glm::vec3 c, p;
	float r;

	c = mid;
	r = max_dist/2;
	float r2 = r*r;
	int n = 0;

	for(int i=0; i<num; i++)
	{
		p=v[i];

		glm::vec3 direction = p-c;
		float dist2 = glm::length2(direction);

		if(dist2 > r2)
		{
			float distance = sqrt(dist2);
			
			float diff = distance - r;

			float diameter = 2*r;
			diameter += diff;
			r = diameter/2;
			r2 = r*r;

			diff /= 2;

			c += diff*direction;
		}
	}
	
	centre = c;
	radius = r;

}

bool BoundingSphere :: checkCollision(BoundingSphere other)
{
	glm::vec3 distance_vec = transformed_centre - other.transformed_centre;
	float distance = glm::length(distance_vec);

	float radius_sum = radius + other.radius;

	if(distance <= radius_sum)
		return true;

	return false;
}

void BoundingSphere :: update(glm::mat4 model_mat)
{
	transformed_centre = glm::vec3(model_mat * glm::vec4(centre, 1.0f));
}


void BoundingSphere :: draw(GLuint spID, glm::vec3 colour)
{
	glm::mat4 sphere_mat = glm::translate(glm::mat4(), transformed_centre);

	int model_matrix_location = glGetUniformLocation (spID, "model");
	glUniformMatrix4fv(model_matrix_location, 1, GL_FALSE, glm::value_ptr(sphere_mat));


	int colour_location = glGetUniformLocation (spID, "colour");
	glUniform3fv(colour_location, 1, glm::value_ptr(colour));

	glutWireSphere(radius, 25, 25);



}
