#ifndef ABSTRACTPARAMETERIZATION_H
#define ABSTRACTPARAMETERIZATION_H

#include "MogsDynamics.h"

class AbstractParameterization
{
    public:

        /// FIXME voir comment mieux faire le compute_kin
        virtual void compute( const double *x , std::vector<MogsDynamics<double> *>& dyns) = 0;

        unsigned int get_nb_param() const
        {
            std::cout<<"nb_param_ = "<<nb_param_ <<std::endl;
            return nb_param_;
        }

        double get_bounds_inf(unsigned int i)
        {
            std::cout<<"i = "<<i <<std::endl;
            std::cout<<"nb_param_ = "<<nb_param_ <<std::endl;
            std::cout<<"bound_inf_.size() = "<< bound_inf_.size()<<std::endl;
            return bound_inf_[i];
        }

        double get_bounds_sup(unsigned int i)
        {
            std::cout<<"i = "<<i <<std::endl;
            std::cout<<"nb_param_ = "<<nb_param_ <<std::endl;
            std::cout<<"bound_sup_.size() = "<< bound_sup_.size()<<std::endl;
            return bound_sup_[i];
        }

        double get_starting_point(unsigned int i)
        {
            return init_[i];
        }
protected:
        unsigned int nb_param_;

        std::vector<double> bound_inf_, bound_sup_, init_;

};

#endif // ABSTRACTPARAMETERIZATION_H
