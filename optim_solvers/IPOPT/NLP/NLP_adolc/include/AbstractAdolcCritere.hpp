// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_adolc.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef ABSTRACTADOLCCRITERE_HPP_INCLUDED
#define ABSTRACTADOLCCRITERE_HPP_INCLUDED

#include "MogsKinematics.h"
#include <adolc.h>
class AbstractAdolcCritere
{
    public:
//     AbstractAdolcCritere ();
//    ~ AbstractAdolcCritere ();

    virtual double compute( const double *x , MogsKinematics<double> * kin) = 0;

    virtual adouble compute( const adouble *x , MogsKinematics<adouble> * kin) = 0;

	double weight_=1;
};


#endif  // ABSTRACTADOLCCRITERIA_HPP_INCLUDED
