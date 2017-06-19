#include <CloseToMiddleFAD_1_4Critere.hpp>

CloseToMiddleFAD_1_4Critere::CloseToMiddleFAD_1_4Critere ( ):CloseToMiddleCriteria( )
{

}

CloseToMiddleFAD_1_4Critere::~CloseToMiddleFAD_1_4Critere ()
{

}

void CloseToMiddleFAD_1_4Critere::init_from_AbstractCriteria(  AbstractCriteria* c)
{
    CloseToMiddleCriteria::init_from_AbstractCriteria(c);
}

void CloseToMiddleFAD_1_4Critere::init_from_xml( QDomElement ctr,
                                                std::vector<MogsOptimDynamics<double> *>& dyns )
{
    CloseToMiddleCriteria::init_from_xml(ctr,dyns);
}


extern "C" CloseToMiddleFAD_1_4Critere* create( )
{
    return new CloseToMiddleFAD_1_4Critere( );
}

extern "C" void destroy(CloseToMiddleFAD_1_4Critere* p)
{
    delete p;
}
