//      MogsIpoptSolver.cpp
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

#include "MogsIpoptSolver.h"
#include <ctime>


// MogsIpoptSolver::MogsIpoptSolver()
// {
//
// }
//
// MogsIpoptSolver::~MogsIpoptSolver()
// {
//     // FIXME
// }

void MogsIpoptSolver::init_problem(AbstractOptimizationProblem** pb)
{
    
}

void MogsIpoptSolver::solve(AbstractOptimizationProblem* pb)
{
    
}

void MogsIpoptSolver::read_solver_option ( )
{
    
}

////void read_problem (const mogs_string & filename);
void MogsIpoptSolver::read_solver_option (QDomElement solver_xml)
{
    // loaded the good type of problem
    QDomElement ElDerivative=solver_xml.firstChildElement("derivative");
    derivative_name_ = ElDerivative.text().simplified();
    qDebug()<<"derivative_name_ = "<< derivative_name_;

 	// Create an instance of the IpoptApplication
	app_ = IpoptApplicationFactory ();

	// read the options
	for (QDomElement childOptions = solver_xml.firstChildElement("ipopt_options"); !childOptions.isNull(); childOptions = childOptions.nextSiblingElement("ipopt_options") )
	{
		mogs_string type = childOptions.attribute("type");
		mogs_string name = childOptions.attribute("name");
		mogs_string value = childOptions.attribute("value");

		if(type =="string")	 		app_->Options()->SetStringValue(name.toStdString().c_str(), value.toStdString().c_str());
		else if (type =="integer")	app_->Options()->SetIntegerValue(name.toStdString().c_str(), value.toInt());
		else if (type =="real")     app_->Options()->SetNumericValue(name.toStdString().c_str(), value.toDouble());
		else	qDebug()<<"option of type : "<< type <<" not defined.";
	}
}

void MogsIpoptSolver::set_option_integer( const mogs_string & option_name,
                                                int value)
{
    app_->Options()->SetIntegerValue(option_name.toStdString().c_str(), value);
}

void MogsIpoptSolver::set_option_real( const mogs_string & option_name,
                                            double value)
{
    app_->Options()->SetNumericValue(option_name.toStdString().c_str(), value);
}

void MogsIpoptSolver::set_option_string( const mogs_string & option_name,
                                                const mogs_string & value)
{
    app_->Options()->SetStringValue(option_name.toStdString().c_str(), value.toStdString().c_str());
}

bool MogsIpoptSolver::solve_ipopt(   SmartPtr < MogsIpoptProblem > nlp )
{
//    nlp_ = (MogsIpoptProblem*) pb;
    std::cout<<"MogsIpoptSolver::solve()"<<std::endl;
	ipopt_status_ = app_->Initialize ();
    std::cout<<"app Initialized"<<std::endl;
	if (ipopt_status_ != Solve_Succeeded)
	  {
		  std::cerr << std::endl << std::endl << "*** Error during initialization!" << std::endl;
		  return false;
	  }

    //set options
    // 	app_->Options()->SetStringValue("derivative_test", "first-order");
	app_->Options()->SetStringValue("hessian_approximation", "limited-memory");

	clock_t begin = clock();
	std::cout<<"before optimization"<<std::endl;
	nlp->print_name();
	ipopt_status_ = app_->OptimizeTNLP (nlp);
	std::cout<<"after optimization"<<std::endl;
	clock_t end = clock();
//	qDebug()<<"Optimization time =" << double(end - begin) / CLOCKS_PER_SEC;

    bool status;
	if (ipopt_status_ == Solve_Succeeded || ipopt_status_ == Solved_To_Acceptable_Level )
	{
	    status = true;
	   // Retrieve some statistics about the solve
	   Ipopt::Index iter_count = app_->Statistics ()->IterationCount ();
//	   std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count<< " iterations!" << std::endl;
	   Number final_obj = app_->Statistics ()->FinalObjective ();
//	   std::cout << std::endl << std::endl <<"*** The final value of the objective function is "<< final_obj << '.' << std::endl;
	}else
	{
	    status = false;
	}
    std::cout<<"MogsIpoptSolver::solve()  done"<<std::endl;
    return status;
}
