#include "StaticPostureParameterization.h"

StaticPostureParameterization::StaticPostureParameterization(   bool compute_forces,
                                                                std::vector<MogsOptimDynamics<double> *>& dyns):StaticPostureParameterization()
{

    compute_forces_ = compute_forces;
    init(dyns);
}

void StaticPostureParameterization::init_from_AbstractParameterization( AbstractParameterization* p)
{
    *this =  *(dynamic_cast<StaticPostureParameterization*>(p));
}

void StaticPostureParameterization::init_from_xml(   QDomElement Param,
                                                     std::vector<MogsOptimDynamics<double> *>& dyns)
{
    plugin_name_ = "StaticPosture";
    compute_forces_ = false;
    std::cout<<"\tConstructor of StaticPostureParameterization"<<std::endl;
    QDomElement cf = Param.firstChildElement("compute_force");
    if( !cf.isNull())
    {
        compute_forces_ = convert_to_bool(cf.text().simplified());
    }
    std::cout<<"\tcompute_forces_ = "<< compute_forces_<<std::endl;
    init(dyns);
}


void StaticPostureParameterization::init(std::vector<MogsOptimDynamics<double> *>& dyns)
{
    nb_robots_ = dyns.size();
    nb_param_ = 0;
    for (int i=0;i<nb_robots_;i++)
    {
        ndofs_.push_back(dyns[i]->getNDof());
        nb_param_ += dyns[i]->getNDof();
    }

    std::cout<<"nb_param_ = "<< nb_param_ <<std::endl;
//    nb_param_ = robot->getNDof();


    /// FIXME, compute the limits
    init_.resize(nb_param_);
    bound_inf_.resize(nb_param_);
    bound_sup_.resize(nb_param_);
    unsigned int cpt = 0;
    for (unsigned int k=0;k<nb_robots_;k++)
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
            cpt ++;

        }
    }
}

StaticPostureParameterization::~StaticPostureParameterization()
{
    //dtor
}
