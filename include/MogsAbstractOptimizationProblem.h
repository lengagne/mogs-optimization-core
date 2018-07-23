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
//	    from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS

#ifndef __MOGSABSTRACTOPTIMIZATIONPROBLEM__
#define __MOGSABSTRACTOPTIMIZATIONPROBLEM__

#include "MogsAbstractProblem.h"
#include "AbstractOptimizationProblem.h"
#include "AbstractOptimizationSolver.h"

#ifdef MogsVisu_FOUND
#include "VisuHolder.h"
#endif

class MogsAbstractOptimizationProblem: public MogsAbstractProblem
{
      public:

        virtual void read_problem (const mogs_string & filename)=0;

        /** Solve the problem	 */
        virtual void solve();

        bool get_status() const
        {
            return status_;
        }

      protected:

        bool status_;

        mogs_string solver_name_;

        AbstractOptimizationProblem* pb_;
        AbstractOptimizationSolver* solver_;

	#ifdef MogsVisu_FOUND
	bool visu_during_optim_;
	VisuHolder * visu_optim_;
	#endif // MogsVisu_FOUND

};

#endif
