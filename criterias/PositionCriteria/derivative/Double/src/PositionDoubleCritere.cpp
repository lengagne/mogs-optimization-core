#include <PositionDoubleCritere.hpp>

PositionDoubleCritere::PositionDoubleCritere ( ):PositionCriteria( )
{

}

PositionDoubleCritere::~PositionDoubleCritere ()
{

}

void PositionDoubleCritere::init_from_AbstractCriteria(AbstractCriteria* c)
{
    PositionCriteria::init_from_AbstractCriteria(c);
}

void PositionDoubleCritere::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    PositionCriteria::init_from_xml(constraint,dyns);
}

extern "C" PositionDoubleCritere* create( )
{
    return new PositionDoubleCritere( );
}

extern "C" void destroy(PositionDoubleCritere* p)
{
    delete p;
}
