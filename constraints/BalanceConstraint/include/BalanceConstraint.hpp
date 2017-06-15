#ifndef  BalanceConstraint_HPP_
#define  BalanceConstraint_HPP_

#include "AbstractConstraint.hpp"

class BalanceConstraint: virtual public AbstractConstraint
{   public:
	BalanceConstraint (QDomElement Constraint,
                          std::vector<MogsOptimDynamics<double> *>& dyns);

    ~BalanceConstraint ();

    void compute(const double* x,double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return compute<double>(x,g, dyns);
    }

    template<typename T>
    void compute(const T*x, T *g, std::vector<MogsOptimDynamics<T> *>& dyns);

    template<typename T>
    void update_dynamics(const T *x, std::vector<MogsOptimDynamics<T> *>& dyns)
    {
        // no param here
    }

        #ifdef MogsVisu_FOUND
        virtual void update_visu (VisuHolder *visu,
                                  std::vector<MogsOptimDynamics<double> *> & dyns,
                                  const double * param);
        #endif // MogsVisu_FOUND

    private:

      QString Robot;
      int n; // number of dof
};

#include "BalanceConstraint.hxx"

#endif
