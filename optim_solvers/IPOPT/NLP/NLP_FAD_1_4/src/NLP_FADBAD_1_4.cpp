//      Problem_Visu.cpp
//      Copyright (C) 2012 lengagne (lengagne@gmail.com) and druon
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
//      from 2012: IUT de Beziers/ LIRMM, Beziers, France
//	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS

#include "NLP_FAD_1_4.hpp"
#include <cassert>
#include <fadiff.h>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include "VisuHolder.h"
#include "MogsProblemClassifier.h"

using namespace Ipopt;

/* Constructor. */
NLP_FAD_1_4::NLP_FAD_1_4 ()
{
    std::cout<<"Constructor of NLP_FAD_1_4"<<std::endl;
}

NLP_FAD_1_4::~NLP_FAD_1_4 ()
{
}

void NLP_FAD_1_4::load_xml( )
{
	kin.SetRobot(robots_[0]);
	akin.SetRobot(robots_[0]);
	MogsProblemClassifier mpc;
	mogs_string library_so;
    QDomElement criteres=root_.firstChildElement("criteres");
    for (QDomElement critere = criteres.firstChildElement ("critere"); !critere.isNull();critere = critere.nextSiblingElement("critere"))
	{

		if (criteres.tagName()=="criteres")
		{
			type=critere.attribute("type");
			name=critere.attribute("name");
			create_FAD_1_4Critere* creator;
			destroy_FAD_1_4Critere* destructor;

			if ( mpc.get_library_plugin("MogsCriteriaNlpFAD_1_4",type,library_so))
			{
				// load the library
				void * library = dlopen(library_so.toAscii(), RTLD_LAZY);
				if (!library) {
					std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load library ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
					exit(0);
				}
				// load the symbols
				creator = (create_FAD_1_4Critere*) dlsym(library, "create");
				destructor = (destroy_FAD_1_4Critere*) dlsym(library, "destroy");
				/// FIXME store the destructor !!
				if (!creator || !destructor)
				{
					std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load symbols of ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
					exit(0);
				}
				// create an instance of the class
				AbstractFAD_1_4Critere* crit = creator(critere,&kin);
				// FIXME for the moment no init from the xml
// 				crit->init(critere);
				std::cout << "name "   <<name.toStdString().c_str() << std::endl;
// 				criteres_.push_back(new CameraFAD_1_4Critere(critere,&kin));
				criteres_.push_back(crit);
			}
			else
			{
				qDebug()<<"Error cannot load the plugin "<<type<<" as an MogsCriteriaNlpFAD_1_4 plugin";
				exit(0);
			}
		}
	}
    QDomElement constraints=root_.firstChildElement("constraints");

    for (QDomElement constraint = constraints.firstChildElement ("constraint"); !constraint.isNull();constraint = constraint.nextSiblingElement("constraint"))
	{

		if (constraints.tagName()=="constraints")
		{
			type=constraint.attribute("type");
			name=constraint.attribute("name");
			create_FAD_1_4Constraint* creator;
			destroy_FAD_1_4Constraint* destructor;
			/// FIXME store the destructor !!

			if ( mpc.get_library_plugin("MogsConstraintNlpFAD_1_4",type,library_so))
			{
				// load the library
				void * library = dlopen(library_so.toAscii(), RTLD_LAZY);
				if (!library) {
					std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load library ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
					exit(0);
				}
				// load the symbols
				creator = (create_FAD_1_4Constraint*) dlsym(library, "create");
				destructor = (destroy_FAD_1_4Constraint*) dlsym(library, "destroy");
				if (!creator || !destructor)
				{
					std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load symbols of ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
					exit(0);
				}
				// create an instance of the class
				AbstractFAD_1_4Constraint* ctr = creator(constraint,&kin);
//                std::cout << "test : " << ctr->get_test() << std::endl;
//                for(int ii = 0; ii < 3; ii++){
//                    std::cout << "Constraint created : g_l["<<ii<<"] :" << ctr->get_lower(ii) << std::endl;
//                    std::cout << "Constraint created : g_u["<<ii<<"] :" << ctr->get_upper(ii) << std::endl;
//                }
				// FIXME for the moment no init from the xml
 				//ctr->init(constraint);
 				//std::cout << "test : " << ctr->get_test() << std::endl;
				std::cout << "loading constraints name "   <<type.toStdString().c_str() << std::endl;
// 				constraints_.push_back(new CameraFAD_1_4constraint(constraint,&kin));
				constraints_.push_back(ctr);
			}
			else
			{
				qDebug()<<"Error cannot load the plugin "<<type<<" as an MogsConstraintNlpFAD_1_4 plugin";
				exit(0);
			}
		}
	}


	std::cout<<"End of load xml"<<std::endl;
}
bool NLP_FAD_1_4::get_nlp_info (Index & n, Index & m, Index & nnz_jac_g,
		     Index & nnz_h_lag, IndexStyleEnum & index_style)
{
    n = kin.getNDof();
    std::cout << "   n = " << n  << std::endl;
    m = 0;
    for(int i=0;i<constraints_.size();i++)
        m += constraints_[i]->get_nb_constraints();
    std::cout << "   m = " << m  << std::endl;
    // detection of the dependency
    Dependency * X = new Dependency [n];
    for(int i=0;i<n;i++)
        X[i].init(i,n);
    Dependency * G = new Dependency [m];
    MogsKinematics<Dependency> k;
    k.SetRobot(kin.model);
    bool computation_done = false;
    std::cout<<"Compute the dependency of the derivative"<<std::endl;
    for (unsigned int i=0; i<constraints_.size(); i++)
    {
        constraints_[i]->compute(X,G,&k,&computation_done);
    }
    for(int i=0;i<m;i++)    for(int j=0;j<n;j++)
    {
        if(G[i].get(j)){
            col_.push_back(j);
            row_.push_back(i);
        }
    }
    std::cout<<"End the dependency of the derivative"<<std::endl;

    nnz_jac_g_ = col_.size();
	nnz_jac_g = nnz_jac_g_;
	std::cout<<"nnz_jac_g = "<< nnz_jac_g <<std::endl;

	nnz_h_lag = 0;
	index_style = TNLP::C_STYLE;
	return true;
}

bool NLP_FAD_1_4::get_bounds_info (Index n, Number * x_l, Number * x_u,
			Index m, Number * g_l, Number * g_u)
{
    assert(n == kin.getNDof());

    Index i, j;
    robots_[0]->getPositionLimit(qmin,qmax);
    // the variables have lower bounds of -qmax
    for (i=0; i<kin.getNDof(); i++)
    {
        x_l[i] = qmin[i];
    }
    // the variables have upper bounds of +qmax
    for (Index i=0; i<kin.getNDof(); i++)
    {
        x_u[i] = qmax[i];
    }
    //added
    unsigned int cpt = 0;
    for (i=0; i<constraints_.size(); i++)
    {
        for (j=0; j<constraints_[i]->get_nb_constraints(); j++)
        {
            g_l[cpt] = constraints_[i]->get_lower(j);
            g_u[cpt] = constraints_[i]->get_upper(j);
            std::cout << "Const["<<cpt<<"] : g_l["<<cpt<<"] :" << g_l[cpt] << std::endl;
            std::cout << "Const["<<cpt<<"] : g_u["<<cpt<<"] :" << g_u[cpt] << std::endl;
            cpt++;
        }
    }

	return true;
}

bool NLP_FAD_1_4::get_starting_point (Index n, bool init_x, Number * x,
			   bool init_z, Number * z_L, Number * z_U,
			   Index m, bool init_lambda, Number * lambda)
{
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    // initialize to the given starting point
    for(int i=0;i<kin.getNDof();i++)
        x[i] = 0.;
	return true;
}

bool NLP_FAD_1_4::eval_f (Index n, const Number * x, bool new_x, Number & obj_value)
{
    int nb = criteres_.size();
    obj_value =0;
	bool mem_kin = false;
    for (int i =0;i<nb;i++)
    {
        Number tmp = criteres_[i]->compute(x,&kin,&mem_kin);
        obj_value+= tmp;
    }

return true;
}

bool NLP_FAD_1_4::eval_grad_f (Index n, const Number * x, bool new_x, Number * grad_f)
{
	// return the gradient of the objective function grad_{x} f(x)
    assert(n == kin.getNDof());
	F<Number>* X = new F<Number>[n];
	for(unsigned int i=0;i<n;i++)
	{
		X[i] = x[i];
		X[i].diff(i,n);
	}
	bool mem_kin = false;
	F<Number> out=0;
	for (int j =0;j<criteres_.size();j++)
		out+=criteres_[j]->compute(X,&akin,&mem_kin);
    for(unsigned int i=0;i<n;i++)
    {
        grad_f[i] = out.d(i);
    }
	return true;
}

bool NLP_FAD_1_4::eval_g (Index n, const Number * x, bool new_x, Index m, Number * g)
{
    bool compute_kin = false;
    assert(n == kin.getNDof());

    m = 3 ;
   for (int i =0;i<constraints_.size();i++)
  {
        constraints_[i]->compute(x,g,&kin,&compute_kin);
        //std::cout << "constraints_["<<i<<"] :" << constraints_[i] << std::endl;
  }
    return true;
}

bool NLP_FAD_1_4::eval_jac_g (Index n, const Number * x, bool new_x,
		   Index m, Index nele_jac, Index * iRow, Index * jCol,
		   Number * values)
{
            assert(n == kin.getNDof());
            //assert(m == constraints_.size());
            m = 3;

            bool compute_kin = false;
            int cpt=0;
            if (values == NULL)
            {
                for (int i=0;i<nele_jac;i++)
                {
                    iRow[i] = row_[i];
                    jCol[i] = col_[i];
                }
            }
           else
           {
                F<Number>* X = new F<Number>[n];
                F<Number>* G = new F<Number>[m];
                for(unsigned int i=0;i<n;i++)
                {
                    X[i] = x[i];
                    X[i].diff(i,n);
                }
                int cpt=0;
                for (unsigned int i=0; i<constraints_.size(); i++)  // for all physical constraints
                {
                    constraints_[i]->compute(X,G,&akin,&compute_kin);
                }

                for (int i=0;i<nele_jac;i++)
                    values[i] = G[row_[i]].d(col_[i]);
           }
	return true;
}

bool NLP_FAD_1_4::eval_h (Index n, const Number * x, bool new_x,
	       Number obj_factor, Index m, const Number * lambda,
	       bool new_lambda, Index nele_hess, Index * iRow,
	       Index * jCol, Number * values)
{
        //assert(n == kin.getNDof());
        //assert(m == constraints_.size());

            if (values == NULL)
            {
            // return the structure. This is a symmetric matrix, fill the lower left
            // triangle only.
            // the Hessian for this problem is actually dense
            }
            else
            {
            // return the values. This is a symmetric matrix, fill the lower left
            // triangle only
            // fill the objective portion
            }

	return true;
}

void NLP_FAD_1_4::finalize_solution (SolverReturn status,
			  Index n, const Number * x, const Number * z_L,
			  const Number * z_U, Index m, const Number * g,
			  const Number * lambda, Number obj_value,
			  const IpoptData * ip_data,
			  IpoptCalculatedQuantities * ip_cq)
{
	save_results(n,x,obj_value);

#ifdef MogsVisu_FOUND
    VisuHolder visu("resultats");

    visu.add("robot",robots_[0]);
	q.resize(robots_[0]->getNDof());
	aq.resize(robots_[0]->getNDof());

    for (int i=0;i<robots_[0]->getNDof();i++)
        q(i) = x[i];

	for (int i=0;i<3;i++)	q(i)= 0;
	std::cout<<"q = "<< q.transpose()<<std::endl;

    visu.apply_q("robot",&q);

    visu.wait_close();
#endif // MogsVisu_FOUND
}




// For dynamic loading of the library : do not remove !!

extern "C" NLP_FAD_1_4* create()
{
    return new NLP_FAD_1_4();
}

extern "C" void destroy(NLP_FAD_1_4* p)
{
    delete p;
}


