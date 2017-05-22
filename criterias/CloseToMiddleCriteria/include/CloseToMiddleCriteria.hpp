#ifndef  CloseToMiddleCriteria_HPP_
#define  CloseToMiddleCriteria_HPP_

#include "AbstractCriteria.hpp"

class CloseToMiddleCriteria: public AbstractCriteria
{   public:
	CloseToMiddleCriteria (QDomElement critere,
                            std::vector<MogsOptimDynamics<double> *>& dyns);

    ~CloseToMiddleCriteria ();

    double compute( std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return compute<double>(dyns);
    }

    template<typename T>
      T compute( std::vector<MogsOptimDynamics<T> *>& dyns);
      private:

        QString Robot;
       Eigen::Matrix<double,Eigen::Dynamic,  1> qm_;
};

#include "CloseToMiddleCriteria.hxx"

#endif // CloseToMiddleCriteria_HPP_
