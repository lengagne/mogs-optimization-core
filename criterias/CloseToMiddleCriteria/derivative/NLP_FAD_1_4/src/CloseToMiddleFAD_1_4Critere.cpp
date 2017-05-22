#include <CloseToMiddleFAD_1_4Critere.hpp>

CloseToMiddleFAD_1_4Critere::CloseToMiddleFAD_1_4Critere (QDomElement critere,
                                                            std::vector<MogsOptimDynamics<double> *>& dyns):CloseToMiddleCriteria(critere,dyns)
{

}

CloseToMiddleFAD_1_4Critere::~CloseToMiddleFAD_1_4Critere ()
{

}

extern "C" CloseToMiddleFAD_1_4Critere* create(QDomElement critere, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    return new CloseToMiddleFAD_1_4Critere(critere, dyns);
}

extern "C" void destroy(CloseToMiddleFAD_1_4Critere* p)
{
    delete p;
}
