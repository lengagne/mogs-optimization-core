//      MogsOptimizationProblem.cpp
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

#include "config_MogsOptimization.h"

#include "MogsOptimizationProblem.h"
#include "MogsAbstractOptimizationNLP.h"

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"


MogsOptimizationProblem::MogsOptimizationProblem()
{
	xsd_file_ = MOGS_PROBLEM_OPTIMIZATION_XSD_FILE;
}

MogsOptimizationProblem::~MogsOptimizationProblem()
{

}

void MogsOptimizationProblem::solve()
{
	std::cout<<" We are solving the MogsOptimizationProblem Problem"<<std::endl;
	
	// Create an instance of your nlp...
	SmartPtr < TNLP > mynlp = new MogsAbstractOptimizationNLP ();

	// Create an instance of the IpoptApplication
	SmartPtr < IpoptApplication > app = IpoptApplicationFactory ();
	
	// Initialize the IpoptApplication and process the options
	ApplicationReturnStatus status;
	status = app->Initialize ();
	if (status != Solve_Succeeded)
	  {
		  std::cout << std::endl << std::
			  endl << "*** Error during initialization!" << std::
			  endl;
		  exit(0);
	  }

	status = app->OptimizeTNLP (mynlp);

	if (status == Solve_Succeeded)
	  {
		  // Retrieve some statistics about the solve
		  Index iter_count = app->Statistics ()->IterationCount ();
		  std::cout << std::endl << std::
			  endl << "*** The problem solved in " << iter_count
			  << " iterations!" << std::endl;

		  Number final_obj = app->Statistics ()->FinalObjective ();
		  std::cout << std::endl << std::
			  endl <<
			  "*** The final value of the objective function is "
			  << final_obj << '.' << std::endl;
	  }
	std::cout << "test IPOPT done" << std::endl;
}


extern "C" MogsOptimizationProblem* create() 
{
    return new MogsOptimizationProblem();
}

extern "C" void destroy(MogsOptimizationProblem* p) 
{
    delete p;
}