#ifndef ABSTRACTPARAMETERIZATION_H
#define ABSTRACTPARAMETERIZATION_H

#include "MogsOptimDynamics.h"
#include "AbstractConstraint.hpp"
#include "AbstractConstraint.hpp"

class AbstractParameterization
{
    public:

        virtual void compute( const double *x , std::vector<MogsOptimDynamics<double> *>& dyns) = 0;

        unsigned int get_nb_param() const
        {
            return nb_param_;
        }

        double get_bounds_inf(unsigned int i) const
        {
            return bound_inf_[i];
        }

        double get_bounds_sup(unsigned int i) const
        {
            return bound_sup_[i];
        }

        QString get_plugin_name() const
        {
            return plugin_name_;
        }

        double get_starting_point(unsigned int i) const
        {
            return init_[i];
        }

        void init_from_constraints(AbstractConstraint* ctr);

        virtual void prepare_computation( std::vector<MogsOptimDynamics<double> *>& dyns) = 0;

        virtual void init_from_AbstractParameterization(  AbstractParameterization* p) = 0;

        virtual void init_from_xml( QDomElement param,
                                    std::vector<MogsOptimDynamics<double> *>& dyns ) = 0;

        virtual void set_init_value(const std::vector<Eigen::Matrix<double,Eigen::Dynamic,1> > q);

protected:
        unsigned int nb_param_;

        std::vector<double> bound_inf_, bound_sup_, init_;

        // name of the parameterization (used to perfom cast)
        QString plugin_name_;
};

#endif // ABSTRACTPARAMETERIZATION_H
