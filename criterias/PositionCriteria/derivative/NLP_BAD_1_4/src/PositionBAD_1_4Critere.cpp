#include <PositionBAD_1_4Critere.hpp>

PositionBAD_1_4Critere::PositionBAD_1_4Critere (QDomElement critere,
                           MogsKinematics<double> *kin):PositionCriteria(critere,kin)
{

}

PositionBAD_1_4Critere::~PositionBAD_1_4Critere ()
{

}

extern "C" PositionBAD_1_4Critere* create(QDomElement critere, MogsKinematics<double> *kin)
{
    return new PositionBAD_1_4Critere(critere, kin);
}

extern "C" void destroy(PositionBAD_1_4Critere* p)
{
    delete p;
}
