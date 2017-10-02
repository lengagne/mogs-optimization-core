#include "AbstractParameterization.h"

void AbstractParameterization::init_from_constraints(AbstractConstraint* ctr)
{
    std::vector<double> inf,sup,init;
    unsigned int nb_param_from_ctr = ctr->get_nb_param(nb_param_);
    ctr->get_param_bound_and_init(inf,sup,init);

    nb_param_ += nb_param_from_ctr;
    // concatenate vectors
    bound_inf_.insert(std::end(bound_inf_), std::begin(inf), std::end(inf));
    bound_sup_.insert(std::end(bound_sup_), std::begin(sup), std::end(sup));
    init_.insert(std::end(init_), std::begin(init), std::end(init));
}

void AbstractParameterization::set_init_value(const std::vector<Eigen::Matrix<double,Eigen::Dynamic,1> > q)
{

}
