#ifndef PositionDFAD_1_4Constraint_HPP_
#define PositionDFAD_1_4Constraint_HPP_

#include "AbstractFAD_1_4Constraint.hpp"
#include "PositionDConstraint.hpp"

class PositionDFAD_1_4Constraint: public AbstractFAD_1_4Constraint, PositionDConstraint
{
 public:
	PositionDFAD_1_4Constraint (QDomElement constraint,
                                std::vector<MogsDynamics<double> *> dyns);

    ~PositionDFAD_1_4Constraint();

    void compute( const Number *x , Number* g,std::vector<MogsDynamics<Number> *> dyns, bool* compute_kin)
    {
        PositionDConstraint::compute<Number>(x,g, dyns, compute_kin);
    }

    void compute( const F<Number>  *x , F<Number>* g, std::vector<MogsDynamics<F<Number>> *> dyns, bool* compute_kin)
    {
        PositionDConstraint::compute<F<Number> >(x,g, dyns, compute_kin);
    }

    void  compute( const Dependency  *x , Dependency* g, std::vector<MogsDynamics<Dependency> *> dyns,bool* compute_kin)
    {
        PositionDConstraint::compute<Dependency>(x,g, dyns, compute_kin);
    }
};
#endif // PositionDFAD_1_4Critere_HPP_INCLUDED
