// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_Double.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef ABSTRACTDoubleCONSTRAINT_HPP_INCLUDED
#define ABSTRACTDoubleCONSTRAINT_HPP_INCLUDED

#include "MogsOptimDynamics.h"
#include "AbstractConstraint.h"

class AbstractDoubleConstraint : virtual public AbstractConstraint
{
    public:

//      From AbstractConstraint
    virtual void compute(const double*x,double *g, std::vector<MogsOptimDynamics<double> *>& dyn) = 0;

    virtual void update_dynamics(const double *x, std::vector<MogsOptimDynamics<double> *> & dyns) = 0;

};

// the types of the class factories
typedef AbstractDoubleConstraint* create_AbstractDoubleConstraint( );

typedef void destroy_DoubleConstraint(AbstractDoubleConstraint*);

#endif  // ABSTRACTDoubleCONSTRAINT_HPP_INCLUDED
