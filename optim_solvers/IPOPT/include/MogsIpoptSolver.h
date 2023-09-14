//      MogsIpoptSolver.h
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

#ifndef __MogsIpoptSolver__
#define __MogsIpoptSolver__

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "MogsIpoptProblem.hpp"
#include "AbstractOptimizationSolver.h"


using namespace Ipopt;

class MogsIpoptSolver : public AbstractOptimizationSolver
{
    public:

    MogsIpoptSolver()
    {
        
    }
// 
//     ~MogsIpoptSolver();

    virtual void init_problem(AbstractOptimizationProblem** pb);

    virtual void solve(AbstractOptimizationProblem* pb);


    void set_option_integer( const mogs_string & option_name,
                             int value);

    void set_option_real( const mogs_string & option_name,
                             double value);

    void set_option_string( const mogs_string & option_name,
                            const mogs_string & value);

    bool solve_ipopt(   SmartPtr < MogsIpoptProblem > nlp );

    protected:
        
        virtual void read_solver_option ( );
        
        virtual void read_solver_option (QDomElement solver_xml);

        mogs_string derivative_name_="not_defined";

        SmartPtr < IpoptApplication > app_;
        ApplicationReturnStatus ipopt_status_;

    private:

};

#endif
