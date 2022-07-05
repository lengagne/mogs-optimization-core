// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_Double.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef ABSTRACTDoubleCRITERE_HPP_INCLUDED
#define ABSTRACTDoubleCRITERE_HPP_INCLUDED

#include "AbstractCriteria.h"

class AbstractDoubleCriteria : virtual public AbstractCriteria
{
    public:
    virtual double compute( std::vector<MogsOptimDynamics<double> *> &dyns) = 0;

};

// the types of the class factories
typedef AbstractDoubleCriteria* create_AbstractDoubleCriteria( );

typedef void destroy_DoubleCriteria(AbstractDoubleCriteria*);

#endif  // ABSTRACTDoubleCRITERIA_HPP_INCLUDED
