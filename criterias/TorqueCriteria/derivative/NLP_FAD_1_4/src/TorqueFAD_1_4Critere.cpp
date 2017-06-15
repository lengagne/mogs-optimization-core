#include <TorqueFAD_1_4Critere.hpp>

TorqueFAD_1_4Critere::TorqueFAD_1_4Critere (QDomElement critere,
                                                            std::vector<MogsOptimDynamics<double> *>& dyns):TorqueCriteria(critere,dyns)
{

}

TorqueFAD_1_4Critere::~TorqueFAD_1_4Critere ()
{

}

extern "C" TorqueFAD_1_4Critere* create(QDomElement critere, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    return new TorqueFAD_1_4Critere(critere, dyns);
}

extern "C" void destroy(TorqueFAD_1_4Critere* p)
{
    delete p;
}
