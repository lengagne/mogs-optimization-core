// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_FAD_1_4.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef ABSTRACTFAD_1_4Parameterization_HPP_INCLUDED
#define ABSTRACTFAD_1_4Parameterization_HPP_INCLUDED

/// FIXME why, we must set #include "MogsOptimDynamics.h" ??
#include "MogsOptimDynamics.h"
#include <fadiff.h>
#include "MogsNlpIpopt.hpp"
#include "AbstractParameterization.h"
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
