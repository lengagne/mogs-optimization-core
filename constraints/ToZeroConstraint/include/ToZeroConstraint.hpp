#ifndef  ToZeroConstraint_HPP_
#define  ToZeroConstraint_HPP_

#include "AbstractConstraint.hpp"

class ToZeroConstraint: virtual public AbstractConstraint
{   public:
	ToZeroConstraint ();

    ~ToZeroConstraint ();

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );

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


    private:

      QString Robot;
      int n; // number of dof
};

#include "ToZeroConstraint.hxx"

#endif
