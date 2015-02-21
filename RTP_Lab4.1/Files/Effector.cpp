#include "Effector.h"

using namespace std;

Effector :: Effector()
{
	position = force_dir = glm::vec3(0.0f);
	force_mag = time = 0.0f;
}



void Effector :: addTBar(TwBar *bar)
{
//	TwAddVarRW(bar, "Effector Position", TW_TYPE_DIR3F, &position, "");
//	TwAddVarCB(bar, "Effector Force Direction", TW_TYPE_DIR3F, setForceDirCB, getForceDirCB, this, "");
	TwAddVarRW(bar, "Effector Force Magnitude", TW_TYPE_FLOAT, &force_mag, "");
//	TwAddVarRW(bar, "Effector Time", TW_TYPE_FLOAT, &time, "");
}


void Effector :: setForceDir(glm::vec3 new_fd)
{
	force_dir = glm::normalize(new_fd);
}

glm::vec3 Effector :: getForceDir() const
{
	return force_dir;
}