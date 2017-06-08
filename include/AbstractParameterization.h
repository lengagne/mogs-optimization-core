#ifndef ABSTRACTPARAMETERIZATION_H
#define ABSTRACTPARAMETERIZATION_H

#include "MogsOptimDynamics.h"
#include "AbstractConstraint.hpp"

class AbstractParameterization
{
    public:

        /// FIXME voir comment mieux faire le compute_kin
        virtual void compute( const double *x , std::vector<MogsOptimDynamics<double> *>& dyns) = 0;

//        virtual void set_param_constraint(std::vector<AbstractFAD_1_4Constraint*> ctr,
//                                          QString Derivative_name);

        unsigned int get_nb_param() const
        {
            return nb_param_;
        }

        double get_bounds_inf(unsigned int i)
        {
            return bound_inf_[i];
        }

        double get_bounds_sup(unsigned int i)
        {
            return bound_sup_[i];
        }

        double get_starting_point(unsigned int i)
        {
            return init_[i];
        }

        unsigned int get_number_constraints()
        {
            return nb_constraints_;
        }

        AbstractConstraint* get_constraint(unsigned int i)
        {
            return (AbstractConstraint*) constraints_[i];
        }

protected:
        unsigned int nb_param_;

        std::vector<double> bound_inf_, bound_sup_, init_;

        unsigned int nb_constraints_ = 0;

        std::vector<AbstractConstraint*> constraints_;

};

#endif // ABSTRACTPARAMETERIZATION_H
