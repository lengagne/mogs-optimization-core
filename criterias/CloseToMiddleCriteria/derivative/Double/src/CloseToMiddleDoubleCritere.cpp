#include <CloseToMiddleDoubleCritere.hpp>

CloseToMiddleDoubleCritere::CloseToMiddleDoubleCritere ( ):CloseToMiddleCriteria( )
{

}

CloseToMiddleDoubleCritere::~CloseToMiddleDoubleCritere ()
{

}

void CloseToMiddleDoubleCritere::init_from_AbstractCriteria(  AbstractCriteria* c)
{
    CloseToMiddleCriteria::init_from_AbstractCriteria(c);
}

void CloseToMiddleDoubleCritere::init_from_xml( QDomElement ctr,
                                                std::vector<MogsOptimDynamics<double> *>& dyns )
{
    CloseToMiddleCriteria::init_from_xml(ctr,dyns);
}


extern "C" CloseToMiddleDoubleCritere* create( )
{
    return new CloseToMiddleDoubleCritere( );
}

extern "C" void destroy(CloseToMiddleDoubleCritere* p)
{
    delete p;
}
