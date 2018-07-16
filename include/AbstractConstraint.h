#ifndef AbstractConstraint_HPP_
#define AbstractConstraint_HPP_

#include "MogsOptimDynamics.h"

#ifdef MogsVisu_FOUND
    #include "VisuHolder.h"
#endif // MogsVisu_FOUND

class AbstractConstraint
{
 public:

        virtual void update_dynamics(const double *x, std::vector<MogsOptimDynamics<double> *> & dyns) = 0;

        virtual void compute( const double *x ,double *g, std::vector<MogsOptimDynamics<double> *> & dyns) = 0;

        virtual void init_from_AbstractConstraint(  AbstractConstraint* c) = 0;

        virtual void init_from_xml( QDomElement ctr,
                                    std::vector<MogsOptimDynamics<double> *>& dyns ) = 0;

        unsigned int get_offset()
        {
            return offset;
        }

        void set_offset(unsigned int offset)
        {
            this->offset = offset;
        }

        unsigned int get_nb_constraints()
        {
            return m;
        }

        double get_upper(int i)
        {
            return upper_[i];
        }

        double get_lower( int i)
        {
            return lower_[i];
        }

        unsigned int get_nb_param(unsigned int offset)
        {
            offset_param_ = offset;
            return nb_param_;
        }

        void get_param_bound_and_init(std::vector<double>& inf,
                                      std::vector<double>& sup,
                                      std::vector<double>& init)
        {
            inf = param_inf_;
            sup = param_sup_;
            init = param_init_;
        }

        QString get_plugin_name() const
        {
            return plugin_name_;
        }

        #ifdef MogsVisu_FOUND
        virtual void update_visu (VisuHolder *visu,
                                  std::vector<MogsOptimDynamics<double> *> & dyns,
                                  const double * param)
        {

        }
        #endif // MogsVisu_FOUND


 protected:

      unsigned int m; /// number of constraints
      std::vector<double> upper_ ;
      std::vector<double> lower_ ;
      unsigned int offset;

      // if the constraint need extra optimization parameters
      unsigned int nb_param_ = 0;
      unsigned int offset_param_ = 0;

      std::vector<double> param_inf_, param_sup_, param_init_;

      QString plugin_name_;

};

// the types of the class factories
typedef AbstractConstraint* create_Constraint( );

typedef void destroy_Constraint(AbstractConstraint*);

#endif // AbstractConstraint_HPP_INCLUDED
