#ifndef StaticPostureFAD_1_4Parameterization_HPP_
#define StaticPostureFAD_1_4Parameterization_HPP_

#include "AbstractFAD_1_4Parameterization.hpp"
#include "StaticPostureParameterization.h"

class StaticPostureFAD_1_4Parameterization: public AbstractFAD_1_4Parameterization, StaticPostureParameterization
{
 public:
	StaticPostureFAD_1_4Parameterization ( );

    ~StaticPostureFAD_1_4Parameterization();


};
#endif // PositionFAD_1_4Critere_HPP_INCLUDED
