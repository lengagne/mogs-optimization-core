#ifndef ABSTRACTPARAMETERIZATION_H
#define ABSTRACTPARAMETERIZATION_H

#include "MogsOptimDynamics.h"

class AbstractParameterization
{
    public:

        /// FIXME voir comment mieux faire le compute_kin
        virtual void compute( const double *x , std::vector<MogsOptimDynamics<double> *>& dyns) = 0;

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
protected:
        unsigned int nb_param_;

        std::vector<double> bound_inf_, bound_sup_, init_;

};

#endif // ABSTRACTPARAMETERIZATION_H
