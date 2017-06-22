#ifndef  TorqueConstraint_HPP_
#define  TorqueConstraint_HPP_

#include "AbstractConstraint.hpp"

class TorqueConstraint: virtual public AbstractConstraint
{   public:
	TorqueConstraint ( );

    ~TorqueConstraint ();

    void compute(const double* x,double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return compute<double>(x,g, dyns);
    }

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );

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

      std::vector<unsigned int> robot_id_;
      std::vector<unsigned int> start_,end_;
};

#include "TorqueConstraint.hxx"

#endif
