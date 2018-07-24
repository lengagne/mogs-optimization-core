#include <TorqueDoubleCritere.hpp>

TorqueDoubleCritere::TorqueDoubleCritere ( ):TorqueCriteria()
{

}

TorqueDoubleCritere::~TorqueDoubleCritere ()
{

}

void TorqueDoubleCritere::init_from_AbstractCriteria(AbstractCriteria* c)
{
    TorqueCriteria::init_from_AbstractCriteria(c);
}

void TorqueDoubleCritere::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    TorqueCriteria::init_from_xml(constraint,dyns);
}


extern "C" TorqueDoubleCritere* create( )
{
    return new TorqueDoubleCritere( );
}

extern "C" void destroy(TorqueDoubleCritere* p)
{
    delete p;
}
