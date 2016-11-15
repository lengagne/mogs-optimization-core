#include <CameraBAD_1_4Critere.hpp>

CameraBAD_1_4Critere::CameraBAD_1_4Critere (QDomElement critere,
                           MogsKinematics<Number> *kin):CameraCriteria(critere,kin)
{

}

CameraBAD_1_4Critere::~CameraBAD_1_4Critere ()
{

}

extern "C" CameraBAD_1_4Critere* create(QDomElement critere, MogsKinematics<Number> *kin)
{
    return new CameraBAD_1_4Critere(critere, kin);
}

extern "C" void destroy(CameraBAD_1_4Critere* p)
{
    delete p;
}
