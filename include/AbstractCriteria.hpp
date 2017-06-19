#ifndef AbstractCriteria_HPP_
#define AbstractCriteria_HPP_

#include "MogsOptimDynamics.h"

class AbstractCriteria
{
 public:
    /// FIXME voir comment mieux faire le compute_kin
    virtual double compute( std::vector<MogsOptimDynamics<double> *>& dyns) = 0;

	double weight_=1;

    QString get_plugin_name() const
    {
        return plugin_name_;
    }

    QString plugin_name_;

};

#endif // AbstractCriteria_HPP_INCLUDED
