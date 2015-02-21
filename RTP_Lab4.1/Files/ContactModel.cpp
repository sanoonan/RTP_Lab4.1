#include "ContactModel.h"

using namespace std;


ContactModel :: ContactModel()
{
	pa = pb = norm = glm::vec3(0.0f);
}

ContactModel :: ContactModel(CollisionPair &_pair, glm::vec3 _pa, glm::vec3 _pb, glm::vec3 _norm)
{
	pair = &_pair;
	pa = _pa;
	pb = _pb;
	norm = _norm;
}


float ContactModel :: calcImpulse(float coeff_rest)
{
	float N, t1, t2, t3, t4;
	float epsilon = coeff_rest;
	RigidBody *A = pair->body1;
	RigidBody *B = pair->body2;

	glm::vec3 va = A->velocity;
	glm::vec3 vb = B->velocity;

	float ma = A->mass;
	float mb = B->mass;

	glm::vec3 xa = A->centre_of_mass;
	glm::vec3 xb = B->centre_of_mass;

	glm::vec3 n = glm::normalize(norm);

	glm::vec3 ra = pa - xa;
	glm::vec3 rb = pb - xb;

	glm::vec3 wa = A->ang_velocity;
	glm::vec3 wb = B->ang_velocity;



	glm::vec3 padot = va + glm::cross(wa, pa - xa);
	glm::vec3 pbdot = vb + glm::cross(wb, pb - xb);

	float vrel = glm::dot(n, padot - pbdot);

	if(vrel <= 0)
		return 0.0f;

	glm::mat3 Ia = A->calcMomentInertia(A->rotation_mat);
	glm::mat3 Iainv = glm::inverse(Ia);

	glm::mat3 Ib = B->calcMomentInertia(B->rotation_mat);
	glm::mat3 Ibinv = glm::inverse(Ib);

	N = -(1 + epsilon) * vrel;

	t1 = 1/ma;
	t2 = 1/mb;



	t3 = glm::dot(n, glm::cross(Iainv * glm::cross(ra, n), ra));
	t4 = glm::dot(n, glm::cross(Ibinv * glm::cross(rb, n), rb));

	float D = t1+t2+t3+t4;

	float j = N/D;
	return j;
}



float ContactModel :: calcLinearImpulse(float coeff_rest)
{
	float N, t1, t2, t3, t4;
	float epsilon = coeff_rest;
	RigidBody *A = pair->body1;
	RigidBody *B = pair->body2;

	glm::vec3 va = A->velocity;
	glm::vec3 vb = B->velocity;

	float ma = A->mass;
	float mb = B->mass;

	glm::vec3 xa = A->centre_of_mass;
	glm::vec3 xb = B->centre_of_mass;

	glm::vec3 n = glm::normalize(norm);


	glm::vec3 padot = va;
	glm::vec3 pbdot = vb;

	float vrel = glm::dot(n, padot - pbdot);


	N = -(1 + epsilon) * vrel;

	t1 = 1/ma;
	t2 = 1/mb;



	t3 = 0.0f;
	t4 = 0.0f;
	float D = t1+t2+t3+t4;

	float j = N/D;
	return j;
}



float ContactModel :: calcAngularImpulse(float coeff_rest)
{
	float N, t1, t2, t3, t4;
	float epsilon = coeff_rest;
	RigidBody *A = pair->body1;
	RigidBody *B = pair->body2;




	glm::vec3 xa = A->centre_of_mass;
	glm::vec3 xb = B->centre_of_mass;

	glm::vec3 n = glm::normalize(norm);

	glm::vec3 ra = pa - xa;
	glm::vec3 rb = pb - xb;

	glm::vec3 wa = A->ang_velocity;
	glm::vec3 wb = B->ang_velocity;

	glm::vec3 padot = glm::cross(wa, ra);
	glm::vec3 pbdot = glm::cross(wb, rb);

	float vrel = glm::dot(n, padot - pbdot);

	glm::mat3 Ia = A->calcMomentInertia(A->rotation_mat);
	glm::mat3 Iainv = glm::inverse(Ia);

	glm::mat3 Ib = B->calcMomentInertia(B->rotation_mat);
	glm::mat3 Ibinv = glm::inverse(Ib);

	N = -(1 + epsilon) * vrel;

	t1 = 0.0f;
	t2 = 0.0f;



	t3 = glm::dot(n, glm::cross(Iainv * glm::cross(ra, n), ra));
	t4 = glm::dot(n, glm::cross(Ibinv * glm::cross(rb, n), rb));

	float D = t1+t2+t3+t4;

	float j = N/D;
	return j;
}

