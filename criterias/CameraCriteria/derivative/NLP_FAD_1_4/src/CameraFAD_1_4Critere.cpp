#include <CameraFAD_1_4Critere.hpp>

CameraFAD_1_4Critere::CameraFAD_1_4Critere (QDomElement critere,
                                            std::vector<MogsOptimDynamics<Number> *>& dyns):CameraCriteria(critere,dyns)
{

}

CameraFAD_1_4Critere::~CameraFAD_1_4Critere ()
{

}

extern "C" CameraFAD_1_4Critere* create(QDomElement critere, std::vector<MogsOptimDynamics<Number> * >& dyns)
{
    return new CameraFAD_1_4Critere(critere, dyns);
}

extern "C" void destroy(CameraFAD_1_4Critere* p)
{
    delete p;
}
