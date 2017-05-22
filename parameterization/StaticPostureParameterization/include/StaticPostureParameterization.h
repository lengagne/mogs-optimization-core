#ifndef STATICPOSTUREPARAMETERIZATION_H
#define STATICPOSTUREPARAMETERIZATION_H

//#include "MogsDynamics.h"
#include <AbstractParameterization.h>

#include <iostream>

class StaticPostureParameterization : virtual public AbstractParameterization
{
    public:
        StaticPostureParameterization(  QDomElement Param,
                                        std::vector<MogsDynamics<double> *>& dyns );

        void compute( const double *x , std::vector<MogsDynamics<double> *>& dyns)
        {
            compute<double>(x,dyns);
        }

        template<typename T>
          void compute( const T *x,std::vector<MogsDynamics<T> *>& dyns);

        virtual ~StaticPostureParameterization();
    protected:
        bool compute_forces_;
};

#include "StaticPostureParameterization.hxx"

#endif // STATICPOSTUREPARAMETERIZATION_H
