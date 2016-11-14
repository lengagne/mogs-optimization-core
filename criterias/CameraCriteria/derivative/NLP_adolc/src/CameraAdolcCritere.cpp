#include <CameraAdolcCritere.hpp>

CameraAdolcCritere::CameraAdolcCritere (QDomElement critere,MogsKinematics<double> *kin):CameraCriteria(critere,kin)
{
	
}

CameraAdolcCritere::~CameraAdolcCritere ()
{
	
}

extern "C" CameraAdolcCritere* create(QDomElement critere, MogsKinematics<double> *kin)
{
    return new CameraAdolcCritere(critere, kin);
}

extern "C" void destroy(CameraAdolcCritere* p)
{
    delete p;
}
