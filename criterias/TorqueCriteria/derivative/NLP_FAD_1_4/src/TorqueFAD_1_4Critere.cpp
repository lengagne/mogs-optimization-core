#include <TorqueFAD_1_4Critere.hpp>

TorqueFAD_1_4Critere::TorqueFAD_1_4Critere ( ):TorqueCriteria()
{

}

TorqueFAD_1_4Critere::~TorqueFAD_1_4Critere ()
{

}

void TorqueFAD_1_4Critere::init_from_AbstractCriteria(AbstractCriteria* c)
{
    TorqueCriteria::init_from_AbstractCriteria(c);
}

void TorqueFAD_1_4Critere::init_from_xml (QDomElement constraint,
                           std::vector<MogsOptimDynamics<double> *>& dyns)
{
    TorqueCriteria::init_from_xml(constraint,dyns);
}


extern "C" TorqueFAD_1_4Critere* create( )
{
    return new TorqueFAD_1_4Critere( );
}

extern "C" void destroy(TorqueFAD_1_4Critere* p)
{
    delete p;
}
