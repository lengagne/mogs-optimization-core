#ifndef ABSTRACTFAD_1_4CRITERE_HPP_INCLUDED
#define ABSTRACTFAD_1_4CRITERE_HPP_INCLUDED

#include <fadiff.h>
#include "AbstractCriteria.h"
#include "NLP_FAD_1_4.hpp"
#include "Dependency.h"

class AbstractFAD_1_4Criteria : virtual public AbstractCriteria
{
    public:

//      From AbstractCriteria
    virtual Number compute( std::vector<MogsOptimDynamics<Number> *> &dyns) = 0;

    virtual F<Number>  compute( std::vector<MogsOptimDynamics<F<Number>>*> & dyns) = 0;

};

// the types of the class factories
typedef AbstractFAD_1_4Criteria* create_FAD_1_4Criteria( );

typedef void destroy_FAD_1_4Criteria(AbstractFAD_1_4Criteria*);

#endif  // ABSTRACTFAD_1_4CRITERIA_HPP_INCLUDED
