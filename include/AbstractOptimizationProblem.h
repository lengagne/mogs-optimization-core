#ifndef __ABSTRACTOPTIMIZATIONPROBLEM__
#define __ABSTRACTOPTIMIZATIONPROBLEM__

#include "MogsOptimDynamics.h"
#include "AbstractParameterization.h"
#include "AbstractCriteria.h"
#include "AbstractConstraint.h"
#include "MogsAbstractProblem.h"

//
class AbstractOptimizationProblem
{
      public:

        double get_obj()
        {
            return obj_value_;
        }

        virtual  void load_xml( )=0;

        void set_robot(MogsRobotProperties* in)
        {
            robots_.push_back(in);
            nb_robots_  = robots_.size();
        }
        
        void set_robots(const std::vector<MogsRobotProperties*> & in)
        {
            robots_ = in;
            nb_robots_  = robots_.size();
        }

        void set_root(QDomElement root)
        {
            root_ = root;
        }

        void set_show_result(bool show_result)
        {
            show_result_= show_result;
        }

        #ifdef MogsVisu_FOUND
        void set_visu( VisuHolder * v,
                      bool during = false)
        {
            visu_optim_ = v;
            visu_during_optim_ = during;
        }
        #endif

      protected:
        QDomElement root_;
        bool show_result_= true;
        double obj_value_;

       	unsigned int nb_robots_;
        std::vector<MogsRobotProperties*> robots_;

        #ifdef MogsVisu_FOUND
        bool visu_during_optim_ = false;
        VisuHolder * visu_optim_;
        #endif // MogsVisu_FOUND

    unsigned int nb_var_;
    unsigned int nb_crit_;
    unsigned int nb_ctr_;
};


#endif
