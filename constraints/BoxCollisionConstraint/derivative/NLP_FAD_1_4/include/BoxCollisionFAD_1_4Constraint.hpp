#ifndef BoxCollisionFAD_1_4Constraint_HPP_
#define BoxCollisionFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "BoxCollisionConstraint.hpp"

class BoxCollisionFAD_1_4Constraint: public AbstractFAD_1_4Constraint, BoxCollisionConstraint
{
 public:
	BoxCollisionFAD_1_4Constraint (QDomElement constraint,
                                std::vector<MogsOptimDynamics<double> *>& dyns);

    ~BoxCollisionFAD_1_4Constraint();

    inline void compute_double(double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        compute(g,dyns);
    }

    void compute(Number* g,std::vector<MogsOptimDynamics<Number> *>& dyns);

    void compute( F<Number>* g, std::vector<MogsOptimDynamics<F<Number>> *>& dyns);

    void compute( Dependency* g, std::vector<MogsOptimDynamics<Dependency> *>& dyns);

};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
