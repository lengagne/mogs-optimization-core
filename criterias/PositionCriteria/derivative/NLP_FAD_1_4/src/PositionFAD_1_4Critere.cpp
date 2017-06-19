#include <PositionFAD_1_4Critere.hpp>

PositionFAD_1_4Critere::PositionFAD_1_4Critere ( ):PositionCriteria( )
{

}

PositionFAD_1_4Critere::~PositionFAD_1_4Critere ()
{

}

void PositionFAD_1_4Critere::init_from_AbstractCriteria(AbstractCriteria* c)
{
    PositionCriteria::init_from_AbstractCriteria(c);
}

void PositionFAD_1_4Critere::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    PositionCriteria::init_from_xml(constraint,dyns);
}

extern "C" PositionFAD_1_4Critere* create( )
{
    return new PositionFAD_1_4Critere( );
}

extern "C" void destroy(PositionFAD_1_4Critere* p)
{
    delete p;
}
