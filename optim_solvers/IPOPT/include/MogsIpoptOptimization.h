//      MogsIpoptOptimization.h
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

#ifndef __MogsIpoptOptimization__
#define __MogsIpoptOptimization__

#include "MogsAbstractOptimization.h"

#include "config_MogsIpoptOptimizationCore.h"

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "MogsNlpIpopt.hpp"
//#include "AbstractAdolcCritere.hpp"


using namespace Ipopt;

// the types of the class factories
typedef MogsNlpIpopt* create_nlp_ipopt();
typedef void destroy_nlp_ipopt(MogsNlpIpopt*);

class MogsIpoptOptimization: public MogsAbstractOptimization
{
    public:

    MogsIpoptOptimization();

    ~MogsIpoptOptimization();

    void init_nlp_problem (const mogs_string & plugin_name);

    void read_problem (const mogs_string & filename);

    void set_robots_to_nlp(const std::vector<MogsRobotProperties*> & in)
    {
        nlp_->set_robots(in);
    }

    void set_problem_properties(const std::vector<MogsOptimDynamics<double>* >& dyns,
                                AbstractParameterization* param,
                                const std::vector<AbstractCriteria* > &criteres,
                                const std::vector<AbstractConstraint*> & constraints)
    {
        nlp_ ->set_problem_properties(dyns,param,criteres,constraints);
    }

    /** Solve the problem	 */
    void solve();

    // solving the problem assuming all the internal values are set.
    void local_solve();

    void set_show_result(bool show_result);


    private:
    SmartPtr < MogsNlpIpopt > nlp_;
    SmartPtr < IpoptApplication > app_;
    create_nlp_ipopt* creator_;
    destroy_nlp_ipopt* destructor_;

};

#endif
