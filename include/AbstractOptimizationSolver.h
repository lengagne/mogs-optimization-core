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

#ifndef __MOGSAbstractOptimizationSolver__
#define __MOGSAbstractOptimizationSolver__

#include "AbstractOptimizationProblem.h"

#ifdef MogsVisu_FOUND
#include "VisuHolder.h"
#endif

class AbstractOptimizationSolver
{
      public:

    virtual void init_problem(AbstractOptimizationProblem** pb) = 0;

    virtual void prepare(AbstractOptimizationProblem* pb);

	/** Solve the problem	 */
    virtual void solve(AbstractOptimizationProblem* pb) = 0;

    virtual void set_robots(const std::vector<MogsRobotProperties*> & in)
    {
        robots_ = in;
        nb_robots_ = robots_.size();
    }

    void set_root(QDomElement root);

     #ifdef MogsVisu_FOUND
     bool need_visu() const
     {
         return visu_during_optim_;
     }

     void set_visu(VisuHolder * v, bool b )
     {
         visu_during_optim_ = b;
         visu_optim_ = v;
     }
     #endif

    #ifdef MogsVisu_FOUND
    bool visu_during_optim_ = false;
    VisuHolder * visu_optim_;
    #endif // MogsVisu_FOUND

protected:
    virtual void read_solver_option () = 0;

    QDomElement solver_xml_,root_;

    std::vector<MogsRobotProperties*> robots_;
    unsigned int nb_robots_;

    bool status_;

};

#endif
