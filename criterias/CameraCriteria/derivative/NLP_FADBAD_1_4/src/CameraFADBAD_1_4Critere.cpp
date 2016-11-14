#include <CameraFADBAD_1_4Critere.hpp>

CameraFADBAD_1_4Critere::CameraFADBAD_1_4Critere (QDomElement critere,
                           MogsKinematics<double> *kin):CameraCriteria(critere,kin)
{

}

CameraFADBAD_1_4Critere::~CameraFADBAD_1_4Critere ()
{

}

extern "C" CameraFADBAD_1_4Critere* create(QDomElement critere, MogsKinematics<double> *kin)
{
    return new CameraFADBAD_1_4Critere(critere, kin);
}

extern "C" void destroy(CameraFADBAD_1_4Critere* p)
{
    delete p;
}
