//      MogsAbstractOptimization.h
//      Copyright (C) 2012 lengagne (lengagne@gmail.com)
//
//      This program is free software: you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation, either version 3 of the License, or
//      (at your option) any later version.
//
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//
//      You should have received a copy of the GNU General Public License
//      along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//      This program was developped in the following labs:
//      2009-2011:  Joint robotics Laboratory - CNRS/AIST,Tsukuba, Japan.
//      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
//      2012-2013:  IUT de Beziers/ LIRMM, Beziers, France
//	from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS

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
