// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_FADBAD_1_4.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef ABSTRACTFADBAD_1_4CRITERE_HPP_INCLUDED
#define ABSTRACTFADBAD_1_4CRITERE_HPP_INCLUDED

#include "MogsKinematics.h"
#include <fadiff.h>
#include "AbstractCriteria.hpp"

class AbstractFADBAD_1_4Critere : public AbstractCriteria
{
    public:
//     AbstractFADBAD_1_4Critere ();
//    ~ AbstractFADBAD_1_4Critere ();

//      From AbstractCriteria
    virtual double compute( const double *x , MogsKinematics<double> * kin,bool* compute_kin) = 0;

    virtual F<double>  compute( const F<double>  *x , MogsKinematics<F<double> > * kin,bool* compute_kin) = 0;

};


#endif  // ABSTRACTFADBAD_1_4CRITERIA_HPP_INCLUDED
