#ifndef ABSTRACTFAD_1_4Parameterization_HPP_INCLUDED
#define ABSTRACTFAD_1_4Parameterization_HPP_INCLUDED

#include "AbstractParameterization.h"
#include <fadiff.h>
#include "MogsIpoptProblem.hpp"
#include "Dependency.h"

class AbstractFAD_1_4Parameterization : virtual public AbstractParameterization
{
    public:
        virtual void compute( const Number *x , std::vector<MogsOptimDynamics<Number>*> & dyns) = 0;

        virtual void compute( const F<Number> *x , std::vector<MogsOptimDynamics<F<Number>>*> & dyns) = 0;

        virtual void compute( const Dependency  *x, std::vector<MogsOptimDynamics<Dependency> *>& dyns) = 0;

        virtual void prepare_computation( std::vector<MogsOptimDynamics<Number> *>& dyns) = 0;

        virtual void prepare_computation( std::vector<MogsOptimDynamics<F<Number>> *>& dyns) = 0;

        virtual void prepare_computation( std::vector<MogsOptimDynamics<Dependency> *>& dyns) = 0;

    protected:

};

// the types of the class factories
typedef AbstractFAD_1_4Parameterization* create_FAD_1_4Parameterization( );

typedef void destroy_FAD_1_4Parameterization(AbstractFAD_1_4Parameterization*);


#endif  // ABSTRACTFAD_1_4Parameterization_HPP_INCLUDED
