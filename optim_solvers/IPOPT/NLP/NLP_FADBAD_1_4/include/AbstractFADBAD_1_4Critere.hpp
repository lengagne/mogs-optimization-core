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

//      From AbstractCriteria
    virtual Number compute( const Number *x , MogsKinematics<Number> * kin,bool* compute_kin) = 0;

    virtual F<Number>  compute( const F<Number>  *x , MogsKinematics<F<Number> > * kin,bool* compute_kin) = 0;

};

// the types of the class factories
typedef AbstractFADBAD_1_4Critere* create_FADBAD_1_4Critere(QDomElement critere,
													MogsKinematics<Number> *kin);
typedef void destroy_FADBAD_1_4Critere(AbstractFADBAD_1_4Critere*);


#endif  // ABSTRACTFADBAD_1_4CRITERIA_HPP_INCLUDED
