// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_BAD_1_4.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef ABSTRACTBAD_1_4CRITERE_HPP_INCLUDED
#define ABSTRACTBAD_1_4CRITERE_HPP_INCLUDED

#include "MogsKinematics.h"
#include "badiff.h"
#include "AbstractCriteria.hpp"

class AbstractBAD_1_4Critere : public AbstractCriteria
{
    public:

//      From AbstractCriteria
    virtual Number compute( const Number *x , MogsKinematics<Number> * kin,bool* compute_kin) = 0;

    virtual B<Number>  compute( const B<Number>  *x , MogsKinematics<B<Number> > * kin,bool* compute_kin) = 0;

};

// the types of the class factories
typedef AbstractBAD_1_4Critere* create_BAD_1_4Critere(QDomElement critere,
													MogsKinematics<Number> *kin);
typedef void destroy_BAD_1_4Critere(AbstractBAD_1_4Critere*);


#endif  // ABSTRACTBAD_1_4CRITERIA_HPP_INCLUDED
