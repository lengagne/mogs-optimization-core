#include <PositionFADBAD_1_4Critere.hpp>

PositionFADBAD_1_4Critere::PositionFADBAD_1_4Critere (QDomElement critere,
                           MogsKinematics<double> *kin):PositionCriteria(critere,kin)
{

}

PositionFADBAD_1_4Critere::~PositionFADBAD_1_4Critere ()
{

}

extern "C" PositionFADBAD_1_4Critere* create(QDomElement critere, MogsKinematics<double> *kin)
{
    return new PositionFADBAD_1_4Critere(critere, kin);
}

extern "C" void destroy(PositionFADBAD_1_4Critere* p)
{
    delete p;
}
