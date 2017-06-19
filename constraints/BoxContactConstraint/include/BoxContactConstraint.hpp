#ifndef  BoxContactConstraint_HPP_
#define  BoxContactConstraint_HPP_

#include "AbstractConstraint.hpp"
#include "BoxCollisionConstraint.hpp"

class BoxContactConstraint: virtual public AbstractConstraint, virtual public BoxCollisionConstraint
{   public:
	BoxContactConstraint ();

    virtual void init_from_AbstractConstraint(  AbstractConstraint* c);

    virtual void init_from_xml( QDomElement ctr,
                                std::vector<MogsOptimDynamics<double> *>& dyns );

    ~BoxContactConstraint ();

    void compute(const double* x, double * g, std::vector<MogsOptimDynamics<double> *>& dyns);

    template<typename T>
    void update_dynamics(const  T *x, std::vector<MogsOptimDynamics<T> *>& dyns);

    template<typename T>
    void compute_contact_constraint( const T*x , T *g, std::vector<MogsOptimDynamics<T> *>& dyns);

    #ifdef MogsVisu_FOUND
    virtual void update_visu (VisuHolder *visu,
                              std::vector<MogsOptimDynamics<double> *> & dyns,
                              const double * param);
    #endif // MogsVisu_FOUND

    protected:
        unsigned int nb_contact_;

        double friction_;

//        unsigned int offset_contact_ctr_;

};

#include "BoxContactConstraint.hxx"

#endif
