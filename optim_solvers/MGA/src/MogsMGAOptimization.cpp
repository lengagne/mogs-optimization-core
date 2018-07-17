//      MogsMGAOptimization.cpp
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

#include "MogsMGAOptimization.h"
#include "MogsProblemClassifier.h"

MogsMGAOptimization::MogsMGAOptimization( )
{
    xsd_file_ = MOGS_MGA_PROBLEM_OPTIMIZATION_XSD_FILE;
}

MogsMGAOptimization::~MogsMGAOptimization()
{
    // FIXME
//    destructor_(&nlp_);
}

void MogsMGAOptimization::read_problem (const mogs_string & filename)
{

   // loaded the good type of problem
    MogsAbstractOptimization::read_problem(filename);
    my_pb_ = new MogsNlpMGA();



	/*
    std::cout<<"find criteres = "<< !criteres.isNull()<<std::endl;
    QDomElement criteres=root_.firstChildElement("criteres");
    for (QDomElement critere = criteres.firstChildElement ("critere"); !critere.isNull(); critere = critere.nextSiblingElement("critere"))
	{

      while(!critere.isNull())
{

        std::cout<<"find one critere"<<std::endl;
        QString Type=critere.attribute("Type","position");
        QString  weight=critere.attribute(" weight","1");
        std::cout << "critere type = " << Type.toStdString().c_str() << std::endl;
        std::cout << "critere weight = " << weight.toStdString().c_str() << std::endl;

        // Get the first child of the component
        QDomElement Child=critere.firstChildElement().toElement();
        QString Body;
        QString Robot;
        QString body_position;
        QString desired_position;
            // Read Name and value

        while (!Child.isNull())
        {
              if (Child.tagName()=="robot")
            {
            Robot=Child.firstChild().toText().data();
            std::cout << "   Robot  = " << Robot.toStdString().c_str() << std::endl;
            }

             if (Child.tagName()=="body")
            {
            Body=Child.firstChild().toText().data();
            std::cout << "   Body  = " << Body.toStdString().c_str() << std::endl;
            }
            if (Child.tagName()=="body_position")
            {body_position=Child.firstChild().toText().data();
            std::cout << "   body_position  = " << body_position.toStdString().c_str() << std::endl;
            }
             if (Child.tagName()=="desired_position")
            {desired_position=Child.firstChild().toText().data();
             std::cout << "   desired_position  = " << desired_position.toStdString().c_str() << std::endl;
            }
           Child = Child.nextSibling().toElement();
        }

        critere = critere.nextSibling().toElement();


  }  } */

//    MogsProblemClassifier mpc;
//    mogs_string plugin_name = "NLP_Adolc";
//    mogs_string library_so;
//    if ( mpc.get_library_plugin("MGA_optimization_nlp",plugin_name,library_so))
//    {
//        // load the library
//        void * library = dlopen(library_so.toAscii(), RTLD_LAZY);
//        if (!library) {
//            std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load library ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
//            exit(0);
//        }
//        // load the symbols
//        creator_ = (create_nlp_MGA*) dlsym(library, "create");
//        destructor_ = (destroy_nlp_MGA*) dlsym(library, "destroy");
//        if (!creator_ || !destructor_)
//        {
//            std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load symbols of ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
//            exit(0);
//        }
//        // create an instance of the class
//        nlp_ = creator_();
//
//    }
//    else
//    {
//        qDebug()<<"Error cannot load the plugin "<<plugin_name<<" as an MGA_optimization_nlp plugin";
//        exit(0);
//    }
//
// 	// Create an instance of the MGAApplication
//	app_ = MGAApplicationFactory ();
}

void MogsMGAOptimization::solve()
{
    std::cout<<"MogsMGAOptimization::solve()"<<std::endl;

    MogsAbstractOptimization::solve();
    #ifdef MogsVisu_FOUND
    my_pb_->set_visu(visu_optim_,visu_during_optim_);
    #endif

	my_pb_->set_robots(robots_);
    my_pb_->set_root(root_);
    my_pb_->load_xml();

    MogsGeneticSolver solver;
    solver.set_nb_queue(10000);
    solver.set_nb_selected(100);
    solver.set_max_iter(100);
    solver.set_search_range_th(1e-6);

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
//    std::cout<<"MogsMGAOptimization::solve()  done"<<std::endl;
}

extern "C" MogsMGAOptimization* create()
{
    return new MogsMGAOptimization();
}

extern "C" void destroy(MogsMGAOptimization* p)
{
    delete p;
}
