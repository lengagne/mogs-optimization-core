#include "StaticPostureParameterization.h"

StaticPostureParameterization::StaticPostureParameterization(   QDomElement Param,
                                                                std::vector<MogsOptimDynamics<double> *>& dyns)
{
    compute_forces_ = false;
    std::cout<<"\tConstructor of StaticPostureParameterization"<<std::endl;
    QDomElement cf = Param.firstChildElement("compute_force");
    if( !cf.isNull())
    {
        compute_forces_ = convert_to_bool(cf.text().simplified());
    }
    std::cout<<"\tcompute_forces_ = "<< compute_forces_<<std::endl;

    for (QDomElement ElContact = Param.firstChildElement("contact"); !ElContact.isNull(); ElContact = ElContact.nextSiblingElement("contact") )
    {
//        QString body1_name = ElBody1.text().simplified();
//        b1.push_back(body1_name);
//        d1_.push_back(new MogsBoxCollisionDefinition(mogs_get_absolute_link(config1),body1_name));
    }

    nb_robots_ = dyns.size();
    nb_param_ = 0;
    for (int i=0;i<nb_robots_;i++)
    {
        ndofs_.push_back(dyns[i]->getNDof());
        nb_param_ += dyns[i]->getNDof();
    }

    std::cout<<"\tnb_param_ = "<< nb_param_ <<std::endl;
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
