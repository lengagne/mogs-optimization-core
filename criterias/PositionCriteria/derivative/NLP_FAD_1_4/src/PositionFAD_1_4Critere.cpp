#include <PositionFAD_1_4Critere.hpp>

PositionFAD_1_4Critere::PositionFAD_1_4Critere (QDomElement critere,
                          std::vector<MogsOptimDynamics<double> *>& dyns):PositionCriteria(critere,dyns)
{

}

PositionFAD_1_4Critere::~PositionFAD_1_4Critere ()
{

}

extern "C" PositionFAD_1_4Critere* create(QDomElement critere,  std::vector<MogsOptimDynamics<double> *>& dyns)
{
    return new PositionFAD_1_4Critere(critere, dyns);
}

extern "C" void destroy(PositionFAD_1_4Critere* p)
{
    delete p;
}
