#include <CloseToMiddleFAD_1_4Critere.hpp>

CloseToMiddleFAD_1_4Critere::CloseToMiddleFAD_1_4Critere (QDomElement critere,
                           MogsKinematics<double> *kin):CloseToMiddleCriteria(critere,kin)
{

}

CloseToMiddleFAD_1_4Critere::~CloseToMiddleFAD_1_4Critere ()
{

}

extern "C" CloseToMiddleFAD_1_4Critere* create(QDomElement critere, MogsKinematics<double> *kin)
{
    return new CloseToMiddleFAD_1_4Critere(critere, kin);
}

extern "C" void destroy(CloseToMiddleFAD_1_4Critere* p)
{
    delete p;
}
