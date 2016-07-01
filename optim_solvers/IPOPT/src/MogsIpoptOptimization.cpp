//      MogsIpoptOptimization.cpp
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

#include "MogsIpoptOptimization.h"

#include "NLP_sample.hpp"

MogsIpoptOptimization::MogsIpoptOptimization()
{
    xsd_file_ = MOGS_IPOPT_PROBLEM_OPTIMIZATION_XSD_FILE;
}

MogsIpoptOptimization::~MogsIpoptOptimization()
{

}

void MogsIpoptOptimization::read_problem (const mogs_string & filename)
{
    // loaded the good type of problem
    nlp_ = new NLP_sample ();
    	// Create an instance of the IpoptApplication
	app_ = IpoptApplicationFactory ();


}

void MogsIpoptOptimization::solve()
{
    std::cout<<"MogsIpoptOptimization::solve()"<<std::endl;
	// Initialize the IpoptApplication and process the options
	ApplicationReturnStatus status;
	status = app_->Initialize ();
	if (status != Solve_Succeeded)
	  {
		  std::cout << std::endl << std::
			  endl << "*** Error during initialization!" << std::
			  endl;
		  return ;
	  }

	status = app_->OptimizeTNLP (nlp_);

	if (status == Solve_Succeeded)
	  {
		  // Retrieve some statistics about the solve
		  Index iter_count = app_->Statistics ()->IterationCount ();
		  std::cout << std::endl << std::
			  endl << "*** The problem solved in " << iter_count
			  << " iterations!" << std::endl;

		  Number final_obj = app_->Statistics ()->FinalObjective ();
		  std::cout << std::endl << std::
			  endl <<
			  "*** The final value of the objective function is "
			  << final_obj << '.' << std::endl;
	  }
    std::cout<<"MogsIpoptOptimization::solve()  done"<<std::endl;
}

extern "C" MogsIpoptOptimization* create()
{
    return new MogsIpoptOptimization();
}

extern "C" void destroy(MogsIpoptOptimization* p)
{
    delete p;
}
