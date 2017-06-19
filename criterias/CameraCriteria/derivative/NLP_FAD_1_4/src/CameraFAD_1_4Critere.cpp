#include <CameraFAD_1_4Critere.hpp>

CameraFAD_1_4Critere::CameraFAD_1_4Critere ( ):CameraCriteria( )
{

}

CameraFAD_1_4Critere::~CameraFAD_1_4Critere ()
{

}

void CameraFAD_1_4Critere::init_from_AbstractCriteria(AbstractCriteria* c)
{
    CameraCriteria::init_from_AbstractCriteria(c);
}

void CameraFAD_1_4Critere::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    CameraCriteria::init_from_xml(constraint,dyns);
}

extern "C" CameraFAD_1_4Critere* create( )
{
    return new CameraFAD_1_4Critere( );
}

extern "C" void destroy(CameraFAD_1_4Critere* p)
{
    delete p;
}
