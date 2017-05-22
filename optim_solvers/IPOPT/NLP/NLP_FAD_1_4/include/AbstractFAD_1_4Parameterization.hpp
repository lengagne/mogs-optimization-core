// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_FAD_1_4.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef ABSTRACTFAD_1_4Parameterization_HPP_INCLUDED
#define ABSTRACTFAD_1_4Parameterization_HPP_INCLUDED

/// FIXME why, we must set #include "MogsDynamics.h" ??
#include "MogsDynamics.h"
#include <fadiff.h>
#include "MogsNlpIpopt.hpp"
#include "AbstractParameterization.h"
#include "Dependency.h"

class AbstractFAD_1_4Parameterization : virtual public AbstractParameterization
{
    public:
        virtual void compute( const Number *x , std::vector<MogsDynamics<Number>*> & dyns) = 0;

        virtual void compute( const F<Number> *x , std::vector<MogsDynamics<F<Number>>*> & dyns) = 0;

        virtual void compute( const Dependency  *x, std::vector<MogsDynamics<Dependency> *>& dyns) = 0;
};

// the types of the class factories
typedef AbstractFAD_1_4Parameterization* create_FAD_1_4Parameterization(QDomElement Param,
                                                                        std::vector<MogsDynamics<double> *>& dyns);

typedef void destroy_FAD_1_4Parameterization(AbstractFAD_1_4Parameterization*);


#endif  // ABSTRACTFAD_1_4Parameterization_HPP_INCLUDED
