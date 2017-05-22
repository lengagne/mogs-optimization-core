#ifndef  CloseToMiddleCriteria_HPP_
#define  CloseToMiddleCriteria_HPP_

#include "AbstractCriteria.hpp"

class CloseToMiddleCriteria: public AbstractCriteria
{   public:
	CloseToMiddleCriteria (QDomElement critere,
                            std::vector<MogsOptimDynamics<double> *>& dyns);

    ~CloseToMiddleCriteria ();

    double compute( const double *x , std::vector<MogsOptimDynamics<double> *>& dyns, bool* compute_kin)
    {
        return compute<double>(x,dyns, compute_kin);
    }

    template<typename T>
      T compute( const T *x,std::vector<MogsOptimDynamics<T> *>& dyns, bool* compute_kin);
      private:

        QString Robot;
       Eigen::Matrix<double,Eigen::Dynamic,  1> qm_;
};

#include "CloseToMiddleCriteria.hxx"

#endif // CloseToMiddleCriteria_HPP_
