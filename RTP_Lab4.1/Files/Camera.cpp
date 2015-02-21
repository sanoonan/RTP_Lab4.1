#include "Camera.h"

using namespace std;

Camera :: Camera()
{
	position = focus = glm::vec3(0.0f);
}

Camera :: Camera(glm::vec3 _position)
{
	position = _position;

}

glm::mat4 Camera :: getRotationMat()
{
	glm::mat4 ret = glm::lookAt(position, focus, glm::vec3(0.0f, 1.0f, 0.0f));
	return ret;
}


void Camera :: moveForwards()
{
	float turnx = atan((position.x - focus.x)/(position.z - focus.z));

	position.x += move_speed*sin(glm::radians(0-turnx));
	position.z -= move_speed*cos(glm::radians(0-turnx));

	focus.x += move_speed*sin(glm::radians(0-turnx));
	focus.z -= move_speed*cos(glm::radians(0-turnx));
}

void Camera :: moveBackwards()
{
	float turnx = atan((position.x - focus.x)/(position.z - focus.z));

	position.x -= move_speed*sin(glm::radians(0-turnx));
	position.z += move_speed*cos(glm::radians(0-turnx));

	focus.x -= move_speed*sin(glm::radians(0-turnx));
	focus.z += move_speed*cos(glm::radians(0-turnx));
}


void Camera :: moveLeft()
{
	float turnx = atan((position.x - focus.x)/(position.z - focus.z));

	position.x -= move_speed*cos(glm::radians(0-turnx));
	position.z -= move_speed*sin(glm::radians(0-turnx));

	focus.x -= move_speed*cos(glm::radians(0-turnx));
	focus.z -= move_speed*sin(glm::radians(0-turnx));
}


void Camera :: moveRight()
{
	float turnx = atan((position.x - focus.x)/(position.z - focus.z));

	position.x += move_speed*cos(glm::radians(0-turnx));
	position.z += move_speed*sin(glm::radians(0-turnx));

	focus.x += move_speed*cos(glm::radians(0-turnx));
	focus.z += move_speed*sin(glm::radians(0-turnx));
}


void Camera :: turnLeft()
{
	float turnx = atan((position.x - focus.x)/(position.z - focus.z));

	focus.x -= move_speed*cos(glm::radians(0-turnx));
	focus.z -= move_speed*sin(glm::radians(0-turnx));
}


void Camera :: turnRight()
{
	float turnx = atan((position.x - focus.x)/(position.z - focus.z));


	focus.x += move_speed*cos(glm::radians(0-turnx));
	focus.z += move_speed*sin(glm::radians(0-turnx));
}

void Camera :: lookUp()
{
	focus.y += move_speed;
}


void Camera :: lookDown()
{
	focus.y -= move_speed;
}



void Camera :: addTBar(TwBar *bar)
{
	TwAddVarRW(bar, "Camera Position", TW_TYPE_DIR3F, &position, "");
	TwAddVarRW(bar, "Camera Focuse", TW_TYPE_DIR3F, &focus, "");
}
