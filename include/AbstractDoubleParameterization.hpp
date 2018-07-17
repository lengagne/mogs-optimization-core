// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_Double.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef DoubleParameterization_HPP_INCLUDED
#define DoubleParameterization_HPP_INCLUDED

/// FIXME why, we must set #include "MogsOptimDynamics.h" ??
#include "AbstractParameterization.h"

class AbstractDoubleParameterization : virtual public AbstractParameterization
{
    public:
        virtual void compute( const double *x , std::vector<MogsOptimDynamics<double>*> & dyns) = 0;

        virtual void prepare_computation( std::vector<MogsOptimDynamics<double> *>& dyns) = 0;


    protected:

};

// the types of the class factories
 typedef AbstractDoubleParameterization* create_AbstractDoubleParameterization( );

 typedef void destroy_DoubleParameterization(AbstractDoubleParameterization*);


#endif  // ABSTRACTDoubleParameterization_HPP_INCLUDED
