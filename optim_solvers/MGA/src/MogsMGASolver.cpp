//      MogsMGASolver.cpp
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
//	    from 2016:  Université Blaise Pascal / axis : ISPR / theme MACCS

#include "MogsMGASolver.h"
#include "MogsProblemClassifier.h"

MogsMGASolver::MogsMGASolver( )
{

}

MogsMGASolver::~MogsMGASolver()
{

}


void MogsMGASolver::init_problem(AbstractOptimizationProblem** pb)
{
    my_pb_ = new MogsMGAProblem();
    std::cout<<"MogsMGASolver::my_pb_ = "<< my_pb_ <<std::endl;
    *pb = (AbstractOptimizationProblem*) my_pb_;
    pb_ = *pb;
    std::cout<<"MogsMGASolver::*pb = "<< *pb <<std::endl;
}


////void read_problem (const mogs_string & filename);
void MogsMGASolver::read_solver_option ()
{
    // For the moment we do not read the options

}



void MogsMGASolver::solve()
{
    std::cout<<"MogsMGASolver::solve()"<<std::endl;

//    MogsAbstractOptimization::solve();
//    #ifdef MogsVisu_FOUND
//    my_pb_->set_visu(visu_optim_,visu_during_optim_);
//    #endif
//
//	my_pb_->set_robots(robots_);
//    my_pb_->set_root(root_);
//    my_pb_->load_xml();

    MogsGeneticSolver solver;
    solver.set_nb_queue(10000);
    solver.set_nb_selected(100);
    solver.set_max_iter(100);
    solver.set_search_range_th(1e-3);

    solver.solve(my_pb_);

//    std::cout<<"il y a "<< robots_url_.size()<<" robots."<<std::endl;
//    qDebug()<<robots_url_[0];
//
//     //donne les fichiers des robots
//     nlp_-> set_robot_url( robots_url_ );
//
//      QDomElement criteres=root_.firstChildElement("criteres");
//
//      nlp_->load_xml(criteres);
//
//	// Initialize the MGAApplication and process the options
//	ApplicationReturnStatus status;
//	status = app_->Initialize ();
//	if (status != Solve_Succeeded)
//	  {
//		  std::cout << std::endl << std::
//			  endl << "*** Error during initialization!" << std::
//			  endl;
//		  return ;
//	  }
//
//    //set options
//
//        app_->Options()->SetStringValue("derivative_test", "first-order");
//// 		app_->Options()->SetNumericValue("derivative_test_perturbation", 1e-3);
// 		app_->Options()->SetNumericValue("tol", 1e-4);
//
//	status = app_->OptimizeTNLP (nlp_);
//
//	if (status == Solve_Succeeded)
//	  {
//		  // Retrieve some statistics about the solve
//		  Index iter_count = app_->Statistics ()->IterationCount ();
//		  std::cout << std::endl << std::
//			  endl << "*** The problem solved in " << iter_count
//			  << " iterations!" << std::endl;
//
//		  Number final_obj = app_->Statistics ()->FinalObjective ();
//		  std::cout << std::endl << std::
//			  endl <<
//			  "*** The final value of the objective function is "
//			  << final_obj << '.' << std::endl;
//	  }
//    std::cout<<"MogsMGASolver::solve()  done"<<std::endl;
}

extern "C" MogsMGASolver* create()
{
    return new MogsMGASolver();
}

extern "C" void destroy(MogsMGASolver* p)
{
    delete p;
}
