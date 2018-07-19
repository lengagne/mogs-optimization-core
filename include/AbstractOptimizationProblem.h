#ifndef __MOGSABSTRACTOPTIMIZATIONPROBLEM__
#define __MOGSABSTRACTOPTIMIZATIONPROBLEM__

#include "MogsOptimDynamics.h"
#include "AbstractParameterization.h"
#include "AbstractCriteria.h"
#include "AbstractConstraint.h"
#include "MogsAbstractProblem.h"

//
class AbstractOptimizationProblem
{
      public:

//        virtual void read_problem (const mogs_string & filename);

//        virtual void set_problem_properties(const std::vector<MogsOptimDynamics<double>* >& dyns,
//                                            AbstractParameterization* param,
//                                            const std::vector<AbstractCriteria* > &criteres,
//                                            const std::vector<AbstractConstraint*> & constraints)=0;

        double get_obj()
        {
            return obj_value_;
        }

        virtual  void load_xml( )=0;

        void set_robots(const std::vector<MogsRobotProperties*> & in)
        {
            robots_ = in;
            nb_robots_  = robots_.size();
            std::cout<<"AbstractOptimizationProblem::nb_robots_ = "<< nb_robots_ <<std::endl;
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
};

// the types of the class factories
typedef AbstractOptimizationProblem* create_optimization_problem();
typedef void destroy_optimization_problem(AbstractOptimizationProblem*);


#endif
