#include <PositionAdolcCritere.hpp>

PositionAdolcCritere::PositionAdolcCritere (QDomElement critere,MogsKinematics<double> *kin):PositionCriteria(critere,kin)
{
	
}

PositionAdolcCritere::~PositionAdolcCritere ()
{
	
}

extern "C" PositionAdolcCritere* create(QDomElement critere, MogsKinematics<double> *kin)
{
    return new PositionAdolcCritere(critere, kin);
}

extern "C" void destroy(PositionAdolcCritere* p)
{
    delete p;
}
