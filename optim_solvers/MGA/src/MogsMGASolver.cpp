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
    *pb = (AbstractOptimizationProblem*) my_pb_;
    pb_ = *pb;
}



void MogsMGASolver::read_solver_option ()
{
    // For the moment we do not read the options
    solver_ = new MogsGeneticSolver();
	for (QDomElement childOptions = solver_xml_.firstChildElement("mga_options"); !childOptions.isNull(); childOptions = childOptions.nextSiblingElement("mga_options") )
	{
		qDebug()<<"We find one option";
		mogs_string type = childOptions.attribute("type");
		mogs_string name = childOptions.attribute("name");
		mogs_string value = childOptions.attribute("value");

		if(type =="integer")
        {
            int v = value.toInt();
            if (name == "nb_queue")
                solver_->set_nb_queue(v);
            else if (name == "nb_selected")
                solver_->set_nb_selected(v);
            else if (name == "max_iter")
                solver_->set_max_iter(v);
        }
		else if (type =="real")
        {
            double v = value.toDouble();
            if (name == "threshold")
                solver_->set_search_range_th(v);
            else if (name == "max_barrier")
                solver_->set_max_barrier(v);
            else if (name == "min_barrier")
                solver_->set_min_barrier(v);

        }
		else	qDebug()<<"option of type : "<< type <<" not defined.";
	}
}



void MogsMGASolver::solve()
{
    std::cout<<"MogsMGASolver::solve()"<<std::endl;
    solver_->solve(my_pb_);
}

extern "C" MogsMGASolver* create()
{
    return new MogsMGASolver();
}

extern "C" void destroy(MogsMGASolver* p)
{
    delete p;
}
