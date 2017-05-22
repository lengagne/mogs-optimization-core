#ifndef STATICPOSTUREPARAMETERIZATION_H
#define STATICPOSTUREPARAMETERIZATION_H

//#include "MogsOptimDynamics.h"
#include <AbstractParameterization.h>

#include <iostream>

class StaticPostureParameterization : virtual public AbstractParameterization
{
    public:
        StaticPostureParameterization(  QDomElement Param,
                                        std::vector<MogsOptimDynamics<double> *>& dyns );

        void compute( const double *x , std::vector<MogsOptimDynamics<double> *>& dyns)
        {
            compute<double>(x,dyns);
        }

        template<typename T>
          void compute( const T *x,std::vector<MogsOptimDynamics<T> *>& dyns);

        virtual ~StaticPostureParameterization();
    protected:
        bool compute_forces_;

        unsigned int nb_robots_;
        std::vector<unsigned int> ndofs_;
};

#include "StaticPostureParameterization.hxx"

#endif // STATICPOSTUREPARAMETERIZATION_H