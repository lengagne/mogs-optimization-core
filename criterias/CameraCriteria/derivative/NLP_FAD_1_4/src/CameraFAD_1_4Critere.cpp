#include <CameraFAD_1_4Critere.hpp>

CameraFAD_1_4Critere::CameraFAD_1_4Critere (QDomElement critere,
                           MogsKinematics<Number> *kin):CameraCriteria(critere,kin)
{

}

CameraFAD_1_4Critere::~CameraFAD_1_4Critere ()
{

}

extern "C" CameraFAD_1_4Critere* create(QDomElement critere, MogsKinematics<Number> *kin)
{
    return new CameraFAD_1_4Critere(critere, kin);
}

extern "C" void destroy(CameraFAD_1_4Critere* p)
{
    delete p;
}
