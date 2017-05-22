#include "StaticPostureParameterization.h"

StaticPostureParameterization::StaticPostureParameterization(   QDomElement Param,
                                                                std::vector<MogsDynamics<double> *>& dyns)
{
    compute_forces_ = false;
    std::cout<<"\tConstructor of StaticPostureParameterization"<<std::endl;
    QDomElement cf = Param.firstChildElement("compute_force");
    if( !cf.isNull())
    {
        compute_forces_ = convert_to_bool(cf.text().simplified());
    }
    std::cout<<"\tcompute_forces_ = "<< compute_forces_<<std::endl;

    unsigned int nb_robots = dyns.size();
    nb_param_ = 0;
    for (int i=0;i<nb_robots;i++)
        nb_param_ += dyns[i]->getNDof();
    std::cout<<"\tnb_param_ = "<< nb_param_ <<std::endl;
//    nb_param_ = robot->getNDof();


    /// FIXME, compute the limits
    init_.resize(nb_param_);
    bound_inf_.resize(nb_param_);
    bound_sup_.resize(nb_param_);
    unsigned int cpt = 0;
    for (unsigned int k=0;k<nb_robots;k++)
    {
        std::vector < double > qmax;
        std::vector < double > qmin;
        dyns[k]->model->getPositionLimit(qmin,qmax);

        for (unsigned int i=0; i<dyns[k]->getNDof(); i++)
        {
            bound_inf_[cpt] = qmin[i];
            bound_sup_[cpt] = qmax[i];

            init_[cpt] = 0;
            if(init_[cpt]<bound_inf_[cpt])  init_[cpt] = bound_inf_[cpt];
            if(init_[cpt]>bound_sup_[cpt])  init_[cpt] = bound_sup_[cpt];

            std::cout<<" param("<<cpt<<") dans [ "<< bound_inf_[cpt]<<" : "<< bound_sup_[cpt] <<" ] "<<std::endl;

            cpt ++;

        }
    }


}

StaticPostureParameterization::~StaticPostureParameterization()
{
    //dtor
}
