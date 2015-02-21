#include "CollisionPair.h"

using namespace std;

CollisionPair :: CollisionPair()
{
	body1 = body2 = NULL;
}

CollisionPair :: CollisionPair(RigidBody &_body1, RigidBody &_body2)
{
	body1 = &_body1;
	body2 = &_body2;
}