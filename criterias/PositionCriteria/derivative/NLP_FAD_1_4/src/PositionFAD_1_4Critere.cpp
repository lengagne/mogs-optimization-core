#include <PositionFAD_1_4Critere.hpp>

PositionFAD_1_4Critere::PositionFAD_1_4Critere (QDomElement critere,
                           MogsKinematics<double> *kin):PositionCriteria(critere,kin)
{

}

PositionFAD_1_4Critere::~PositionFAD_1_4Critere ()
{

}

extern "C" PositionFAD_1_4Critere* create(QDomElement critere, MogsKinematics<double> *kin)
{
    return new PositionFAD_1_4Critere(critere, kin);
}

extern "C" void destroy(PositionFAD_1_4Critere* p)
{
    delete p;
}
