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

#define PRINT 1


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
    #ifdef PRINT
    std::cout<<"start load_xml"<<std::endl;
    #endif // PRINT
    for (int i=0;i<nb_robots_;i++)
    {
        dyns_.push_back( new MogsDynamics<double>(robots_[i]));
        adyns_.push_back( new MogsDynamics<F<double>>(robots_[i]));
    }
//	kin.SetRobot(robots_[0]);
//	akin.SetRobot(robots_[0]);
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
				AbstractFAD_1_4Critere* crit = creator(critere,dyns_);
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
				AbstractFAD_1_4Constraint* ctr = creator(constraint,dyns_);
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


    /// FIXME allow to change the type of the AbstractParameterization through plugins
    QDomElement param =root_.firstChildElement("parameterization");

    if (param.tagName()=="parameterization")
    {
//        type = "StaticPosture";
        type=param.attribute("type");

        create_FAD_1_4Parameterization* creator;
        destroy_FAD_1_4Parameterization* destructor;
        /// FIXME store the destructor !!

        if ( mpc.get_library_plugin("MogsParameterizationNlpFAD_1_4",type,library_so))
        {
            // load the library
            void * library = dlopen(library_so.toAscii(), RTLD_LAZY);
            if (!library) {
                std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load library ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
                exit(0);
            }
            // load the symbols
            creator = (create_FAD_1_4Parameterization*) dlsym(library, "create");
            destructor = (destroy_FAD_1_4Parameterization*) dlsym(library, "destroy");
            if (!creator || !destructor)
            {
                std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load symbols of ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
                exit(0);
            }
            // create an instance of the class
            parameterization_ = creator( param, dyns_ );
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
//            parameterizations_.push_back(prm);
        }
        else
        {
            qDebug()<<"Error cannot load the plugin "<<type<<" as an MogsParameterizationNlpFAD_1_4 plugin";
            exit(0);
        }
    }else
    {
        std::cerr<<"ERROR cannot find balise parameterization"<<std::endl;
        exit(0);
    }
    std::cout<<"parameterization_ = "<<  parameterization_<< std::endl;

    #ifdef PRINT
    std::cout<<"end of load_xml"<<std::endl;
    #endif // PRINT
}
bool NLP_FAD_1_4::get_nlp_info (Index & n, Index & m, Index & nnz_jac_g,
		     Index & nnz_h_lag, IndexStyleEnum & index_style)
{
    #ifdef PRINT
    std::cout<<"start get_nlp_info"<<std::endl;
    #endif // PRINT
    std::cout<<"parameterization_ = "<<  parameterization_<< std::endl;
    std::cout<<"nb_param = "<<  parameterization_->get_nb_param()<< std::endl;
    nb_var_= parameterization_->get_nb_param();

    n = nb_var_;
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
    std::vector< MogsDynamics<Dependency> *> k;
    for(int i=0;i<nb_robots_;i++)
        k.push_back(new MogsDynamics<Dependency>(dyns_[i]->model));
    bool computation_done = false;
    std::cout<<"Compute the dependency of the derivative"<<std::endl;

    parameterization_->compute(X,k);

    for (unsigned int i=0; i<constraints_.size(); i++)
    {
        constraints_[i]->compute(X,G,k,&computation_done);
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

	// destroy the object
	for(int i=0;i<nb_robots_;i++)
        delete k[i];

    #ifdef PRINT
    std::cout<<"end of get_nlp_info"<<std::endl;
    #endif // PRINT

	return true;
}

bool NLP_FAD_1_4::get_bounds_info (Index n, Number * x_l, Number * x_u,
			Index m, Number * g_l, Number * g_u)
{
    #ifdef PRINT
    std::cout<<"start get_bounds_info"<<std::endl;
    #endif // PRINT
    assert(n == nb_var_);

    Index i, j;
    unsigned int cpt = 0;
    std::cout<<"parameterization_ = "<<  parameterization_<< std::endl;
    std::cout<<"nb_param = "<<  parameterization_->get_nb_param()<< std::endl;


    for (int i=0;i<n;i++)
    {
        std::cout<<"i = "<< i<<"  parameterization_ = "<< parameterization_<< std::endl;
        x_l[i] = parameterization_->get_bounds_inf(i);
        x_u[i] = parameterization_->get_bounds_sup(i);
    }
    std::cout<<"fin "<< std::endl;

/*    for (int k=0;k<nb_robots_;k++)
    {
        robots_[k]->getPositionLimit(qmin_[k],qmax_[k]);
        // the variables have lower bounds of -qmax_
        for (i=0; i<dyns_[k]->getNDof(); i++)
        {
            x_l[cpt] = qmin_[k][i];
            x_u[cpt++] = qmax_[k][i];
        }
    }*/
    //added
    cpt = 0;
    for (i=0; i<constraints_.size(); i++)
    {
        for (j=0; j<constraints_[i]->get_nb_constraints(); j++)
        {
            g_l[cpt] = constraints_[i]->get_lower(j);
            g_u[cpt] = constraints_[i]->get_upper(j);
//            std::cout << "Const["<<cpt<<"] : g_l["<<cpt<<"] :" << g_l[cpt] << std::endl;
//            std::cout << "Const["<<cpt<<"] : g_u["<<cpt<<"] :" << g_u[cpt] << std::endl;
            cpt++;
        }
    }
    #ifdef PRINT
    std::cout<<"end of get_bounds_info"<<std::endl;
    #endif // PRINT
	return true;
}

bool NLP_FAD_1_4::get_starting_point (Index n, bool init_x, Number * x,
			   bool init_z, Number * z_L, Number * z_U,
			   Index m, bool init_lambda, Number * lambda)
{
    #ifdef PRINT
    std::cout<<"start get_starting_point"<<std::endl;
    #endif // PRINT
    std::cout<<"parameterization_ = "<<  parameterization_<< std::endl;
    std::cout<<"nb_param = "<<  parameterization_->get_nb_param()<< std::endl;

    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);


    // initialize to the given starting point
    for(int i=0;i<nb_var_;i++)
        x[i] = 0.;
        //x[i] = parameterization_->get_starting_point(i);

    #ifdef PRINT
    std::cout<<"end of get_starting_point"<<std::endl;
    #endif // PRINT

	return true;
}

bool NLP_FAD_1_4::eval_f (Index n, const Number * x, bool new_x, Number & obj_value)
{
    #ifdef PRINT
    std::cout<<"start eval_f"<<std::endl;
    #endif // PRINT
    int nb = criteres_.size();
    obj_value =0;

    parameterization_->compute(x,dyns_);

	bool mem_kin = false;
    for (int i =0;i<nb;i++)
    {
        Number tmp = criteres_[i]->compute(x,dyns_,&mem_kin);
        obj_value+= tmp;
    }
    #ifdef PRINT
    std::cout<<"end of eval_f"<<std::endl;
    #endif // PRINT
    return true;
}

bool NLP_FAD_1_4::eval_grad_f (Index n, const Number * x, bool new_x, Number * grad_f)
{
    #ifdef PRINT
    std::cout<<"start eval_grad_f"<<std::endl;
    #endif // PRINT
	// return the gradient of the objective function grad_{x} f(x)
    assert(n == nb_var_);
	F<Number>* X = new F<Number>[n];
	for(unsigned int i=0;i<n;i++)
	{
		X[i] = x[i];
		X[i].diff(i,n);
	}

	parameterization_->compute(X,adyns_);

	bool mem_kin = false;
	F<Number> out=0;
	for (int j =0;j<criteres_.size();j++)
		out+=criteres_[j]->compute(X,adyns_,&mem_kin);
    for(unsigned int i=0;i<n;i++)
    {
        grad_f[i] = out.d(i);
    }
    #ifdef PRINT
    std::cout<<"end of eval_grad_f"<<std::endl;
    #endif // PRINT
	return true;
}

bool NLP_FAD_1_4::eval_g (Index n, const Number * x, bool new_x, Index m, Number * g)
{
    #ifdef PRINT
    std::cout<<"start eval_g"<<std::endl;
    #endif // PRINT
    bool compute_kin = false;
    assert(n == nb_var_);

    parameterization_->compute(x,dyns_);

    m = 3 ;
    for (int i =0;i<constraints_.size();i++)
    {
        constraints_[i]->compute(x,g,dyns_,&compute_kin);
        //std::cout << "constraints_["<<i<<"] :" << constraints_[i] << std::endl;
    }
    #ifdef PRINT
    std::cout<<"end of eval_g"<<std::endl;
    #endif // PRINT
    return true;
}

bool NLP_FAD_1_4::eval_jac_g (Index n, const Number * x, bool new_x,
		   Index m, Index nele_jac, Index * iRow, Index * jCol,
		   Number * values)
{
    #ifdef PRINT
    std::cout<<"start eval_jac_g"<<std::endl;
    #endif // PRINT
    assert(n == nb_var_);
    //assert(m == constraints_.size());

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

        parameterization_->compute(X,adyns_);

        int cpt=0;
        for (unsigned int i=0; i<constraints_.size(); i++)  // for all physical constraints
        {
            constraints_[i]->compute(X,G,adyns_,&compute_kin);
        }

        for (int i=0;i<nele_jac;i++)
            values[i] = G[row_[i]].d(col_[i]);
    }
    #ifdef PRINT
    std::cout<<"end of eval_jac_g"<<std::endl;
    #endif // PRINT
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
    q.resize(nb_robots_);
    aq.resize(nb_robots_);
    for(int k=0;k<nb_robots_;k++)
    {
        visu.add(robots_[k]->getRobotName(),robots_[k]);
        q[k].resize(robots_[k]->getNDof());
        aq[k].resize(robots_[k]->getNDof());

    }

    int cpt = 0;
    for(int k=0;k<nb_robots_;k++)
    {
        for (int i=0;i<robots_[k]->getNDof();i++)
            q[k](i) = x[cpt++];

        std::cout<<"q["<<k<<"] = "<< q[k].transpose()<<std::endl;

        visu.apply_q(robots_[k]->getRobotName(),&q[k]);

    }

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


