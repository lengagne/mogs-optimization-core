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
#include "MogsProblemClassifier.h"
#include "AbstractLoader.h"

//#define PRINT 1


using namespace Ipopt;

/* Constructor. */
NLP_FAD_1_4::NLP_FAD_1_4 ()
{
    #ifdef PRINT
    std::cout<<"Constructor of NLP_FAD_1_4"<<std::endl;
    #endif
	compute_number_ = false;
	compute_gradient_ = false;

}

NLP_FAD_1_4::~NLP_FAD_1_4 ()
{
}

void NLP_FAD_1_4::load_ctrs_crits(std::vector<QDomElement> & ctrs,
                                  std::vector<QDomElement> & crits)
{
    AbstractLoader loader;
    for (unsigned int i=0;i<crits.size();i++)
    {
        AbstractFAD_1_4Critere* crit = dynamic_cast<AbstractFAD_1_4Critere*> (loader.get_criteria<create_FAD_1_4Critere*>("MogsCriteriaNlpFAD_1_4",crits[i],dyns_));
        criteres_.push_back(crit);
    }

    for (unsigned int i=0;i<ctrs.size();i++)
	{
        AbstractFAD_1_4Constraint* ctr = dynamic_cast<AbstractFAD_1_4Constraint*> (loader.get_constraint<create_FAD_1_4Constraint*>("MogsConstraintNlpFAD_1_4",ctrs[i],dyns_));

        // std::cout << "loading constraints name "   <<constraint.attribute("type").toStdString().c_str() << std::endl;
        constraints_.push_back(ctr);
	}
}

void NLP_FAD_1_4::load_xml( )
{
    #ifdef PRINT
    std::cout<<"start load_xml"<<std::endl;
    #endif // PRINT
    for (int i=0;i<nb_robots_;i++)
    {
        dyns_.push_back( new MogsOptimDynamics<double>(robots_[i]));
        adyns_.push_back( new MogsOptimDynamics<F<double>>(robots_[i]));
    }
    AbstractLoader loader;

	MogsProblemClassifier mpc;
	mogs_string library_so;
    QDomElement criteres=root_.firstChildElement("criteres");
    for (QDomElement critere = criteres.firstChildElement ("critere"); !critere.isNull();critere = critere.nextSiblingElement("critere"))
	{
        AbstractFAD_1_4Critere* crit = dynamic_cast<AbstractFAD_1_4Critere*> (loader.get_criteria<create_FAD_1_4Critere*>("MogsCriteriaNlpFAD_1_4",critere,dyns_));
        criteres_.push_back(crit);
	}

    /// FIXME allow to change the type of the AbstractParameterization through plugins
    QDomElement param =root_.firstChildElement("parameterization");
    if (param.tagName()=="parameterization")
    {
        parameterization_ = dynamic_cast<AbstractFAD_1_4Parameterization*> (loader.get_parameterization<create_FAD_1_4Parameterization*>("MogsParameterizationNlpFAD_1_4",param,dyns_));
    }else
    {
        std::cerr<<"ERROR cannot find balise parameterization"<<std::endl;
        exit(0);
    }

    QDomElement constraints=root_.firstChildElement("constraints");

    for (QDomElement constraint = constraints.firstChildElement ("constraint"); !constraint.isNull();constraint = constraint.nextSiblingElement("constraint"))
	{
        AbstractFAD_1_4Constraint* ctr = dynamic_cast<AbstractFAD_1_4Constraint*> (loader.get_constraint<create_FAD_1_4Constraint*>("MogsConstraintNlpFAD_1_4",constraint,dyns_));

        // std::cout << "loading constraints name "   <<constraint.attribute("type").toStdString().c_str() << std::endl;
        constraints_.push_back(ctr);
	}

    #ifdef PRINT
    std::cout<<"end of load_xml"<<std::endl;
    #endif // PRINT
}
bool NLP_FAD_1_4::get_nlp_info (Ipopt::Index & n, Ipopt::Index & m, Ipopt::Index & nnz_jac_g,
		     Ipopt::Index & nnz_h_lag, IndexStyleEnum & index_style)
{
    #ifdef PRINT
    std::cout<<"start get_nlp_info"<<std::endl;
    #endif // PRINT
    nb_robots_ = robots_.size();

    nb_ctr_ = constraints_.size();
    for(int i=0;i<nb_ctr_;i++)
        parameterization_->init_from_constraints(constraints_[i]);

    nb_var_= parameterization_->get_nb_param();

    n = nb_var_;
    #ifdef PRINT
    std::cout << "   n = " << n  << std::endl;
    #endif

    m = 0;
    for(int i=0;i<nb_ctr_;i++)
    {
        constraints_[i]->set_offset(m);
        m += constraints_[i]->get_nb_constraints();
    }

    #ifdef PRINT
    std::cout << "   m = " << m  << std::endl;
    #endif // PRINT

    // detection of the dependency
    Dependency * DX = new Dependency [n];
    for(int i=0;i<n;i++)
        DX[i].init(i,n);

    Dependency * DG = new Dependency [m];
    std::vector< MogsOptimDynamics<Dependency> *> k;
    for(int i=0;i<nb_robots_;i++)
        k.push_back(new MogsOptimDynamics<Dependency>(robots_[i]));
    parameterization_->prepare_computation(k);
    for (unsigned int i=0; i<nb_ctr_; i++)
        constraints_[i]->update_dynamics(DX,k);

    parameterization_->compute(DX,k);

    for (unsigned int i=0; i<nb_ctr_; i++)
        constraints_[i]->compute(DX,DG,k);

    for(int i=0;i<m;i++)    for(int j=0;j<n;j++)
    {
//        std::cout<<"G["<<i<<"].get("<<j<<") = "<< G[i].get(j)<<std::endl;
        if(DG[i].get(j)){
            col_.push_back(j);
            row_.push_back(i);
        }
    }
    #ifdef PRINT
    std::cout<<"End the dependency of the derivative"<<std::endl;
    #endif // PRINT

    nnz_jac_g_ = col_.size();
	nnz_jac_g = nnz_jac_g_;
	#ifdef PRINT
	std::cout<<"nnz_jac_g = "<< nnz_jac_g <<std::endl;
    #endif // PRINT

	nnz_h_lag = 0;
	index_style = TNLP::C_STYLE;

	// destroy the object
	for(int i=0;i<nb_robots_;i++)
        delete k[i];

	G = new F<Number>[m];
	X = new F<Number>[n];

    #ifdef PRINT
    std::cout<<"end of get_nlp_info"<<std::endl;
    #endif // PRINT

    #ifdef MogsVisu_FOUND
    if(visu_during_optim_ && !visu_optim_)
    {
        visu_optim_ = new VisuHolder("intermediate result");
        for(int k=0;k<nb_robots_;k++)
            visu_optim_->add(robots_[k]->getRobotName(),robots_[k]);
    }
    #endif // MogsVisu_FOUND

	return true;
}

bool NLP_FAD_1_4::get_bounds_info (Ipopt::Index n, Number * x_l, Number * x_u,
			Ipopt::Index m, Number * g_l, Number * g_u)
{
    #ifdef PRINT
    std::cout<<"start get_bounds_info"<<std::endl;
    #endif // PRINT
    assert(n == nb_var_);

    Ipopt::Index i, j;
    unsigned int cpt = 0;
    for (int i=0;i<n;i++)
    {
        x_l[i] = parameterization_->get_bounds_inf(i);
        x_u[i] = parameterization_->get_bounds_sup(i);
//        std::cout<<"bounds("<<i<<") = "<< x_l[i]<<" : "<< x_u[i]<<std::endl;
    }

    cpt = 0;
    for (i=0; i<constraints_.size(); i++)
    {
//        qDebug()<<"dealing with constraint "<< constraints_[i];
//        qDebug()<<"dealing with constraint of type : "<< constraints_[i]->get_plugin_name();
        for (j=0; j<constraints_[i]->get_nb_constraints(); j++)
        {
            g_l[cpt] = constraints_[i]->get_lower(j);
            g_u[cpt] = constraints_[i]->get_upper(j);
//            std::cout<<"g("<<cpt<<") in "<< g_l[cpt]<<" : "<< g_u[cpt]<<std::endl;
            cpt++;
        }
    }
    #ifdef PRINT
    std::cout<<"end of get_bounds_info"<<std::endl;
    #endif // PRINT
	return true;
}

bool NLP_FAD_1_4::get_starting_point (Ipopt::Index n, bool init_x, Number * x,
			   bool init_z, Number * z_L, Number * z_U,
			   Ipopt::Index m, bool init_lambda, Number * lambda)
{
    #ifdef PRINT
    std::cout<<"start get_starting_point"<<std::endl;
    #endif // PRINT
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    // initialize to the given starting point
    for(int i=0;i<nb_var_;i++)
        x[i] = parameterization_->get_starting_point(i);

//    for(int i=0;i<nb_var_;i++)
//        std::cout<<"init("<<i<<") = "<< x[i]<<std::endl;

    #ifdef PRINT
    std::cout<<"end of get_starting_point"<<std::endl;
    #endif // PRINT

	return true;
}

bool NLP_FAD_1_4::eval_f (Ipopt::Index n, const Number * x, bool new_x, Number & obj_value)
{
    #ifdef PRINT
    std::cout<<"start eval_f"<<std::endl;
    #endif // PRINT
    run_computation(x,n,new_x);
    int nb = criteres_.size();
    obj_value =0;
    for (int i =0;i<nb;i++)
    {
        Number tmp = criteres_[i]->compute(dyns_);
        obj_value+= tmp;
    }

   #ifdef MogsVisu_FOUND
    if(visu_during_optim_)
    {
        for(int k=0;k<nb_robots_;k++)
        {
            visu_optim_->apply_q(robots_[k]->getRobotName(),&dyns_[k]->q_);
        }

        visu_optim_-> clear_lines();
        for (int i=0;i<constraints_.size();i++)
            constraints_[i]->update_visu(visu_optim_,dyns_,(const double*) x);
    }
    #endif // MogsVisu_FOUND

    #ifdef PRINT
    std::cout<<"end of eval_f"<<std::endl;
    #endif // PRINT
    return true;
}

bool NLP_FAD_1_4::eval_grad_f (Ipopt::Index n, const Number * x, bool new_x, Number * grad_f)
{
    #ifdef PRINT
    std::cout<<"start eval_grad_f"<<std::endl;
    #endif // PRINT
    run_gradient_computation(x,n,new_x);
	F<Number> out=0;
	for (int j =0;j<criteres_.size();j++)
		out+=criteres_[j]->compute(adyns_);
    for(unsigned int i=0;i<n;i++)
    {
        grad_f[i] = out.d(i);
    }
    #ifdef PRINT
    std::cout<<"end of eval_grad_f"<<std::endl;
    #endif // PRINT
	return true;
}

bool NLP_FAD_1_4::eval_g (Ipopt::Index n, const Number * x, bool new_x, Ipopt::Index m, Number * g)
{
    #ifdef PRINT
    std::cout<<"start eval_g"<<std::endl;
    #endif // PRINT
    assert(n == nb_var_);
    run_computation(x,n,new_x);

    for (int i =0;i<nb_ctr_;i++)
        constraints_[i]->compute(x,g,dyns_);

    #ifdef PRINT
    std::cout<<"end of eval_g"<<std::endl;
    #endif // PRINT
    return true;
}

bool NLP_FAD_1_4::eval_jac_g (Ipopt::Index n, const Number * x, bool new_x,
		   Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index * iRow, Ipopt::Index * jCol,
		   Number * values)
{
    #ifdef PRINT
    std::cout<<"start eval_jac_g"<<std::endl;
    #endif // PRINT
    assert(n == nb_var_);
    //assert(m == constraints_.size());

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
        run_gradient_computation(x,n,new_x);
        for (unsigned int i=0; i<nb_ctr_; i++)  // for all physical constraints
            constraints_[i]->compute(X,G,adyns_);

        unsigned int cpt=0;
        for (int i=0;i<nele_jac;i++)
            values[i] = G[row_[i]].d(col_[i]);
    }
    #ifdef PRINT
    std::cout<<"end of eval_jac_g"<<std::endl;
    #endif // PRINT

	return true;
}

bool NLP_FAD_1_4::eval_h (Ipopt::Index n, const Number * x, bool new_x,
	       Number obj_factor, Ipopt::Index m, const Number * lambda,
	       bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index * iRow,
	       Ipopt::Index * jCol, Number * values)
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
			  Ipopt::Index n, const Number * x, const Number * z_L,
			  const Number * z_U, Ipopt::Index m, const Number * g,
			  const Number * lambda, Number obj_value,
			  const IpoptData * ip_data,
			  IpoptCalculatedQuantities * ip_cq)
{
    obj_value_ = obj_value;
    run_computation(x,n,true);
	save_results(n,x,obj_value);

//    for (int i=0;i<n;i++)
//        std::cout<<"x("<<i<<") = "<< x[i]<<std::endl;
//
//    for (int i=0;i<m;i++)
//        std::cout<<"g("<<i<<") = "<< g[i]<<std::endl;

#ifdef MogsVisu_FOUND

    q.resize(nb_robots_);
    aq.resize(nb_robots_);
    for(int k=0;k<nb_robots_;k++)
    {
       visu_optim_->add(robots_[k]->getRobotName(),robots_[k]);
        q[k].resize(robots_[k]->getNDof());
        aq[k].resize(robots_[k]->getNDof());

    }
    parameterization_->prepare_computation(dyns_);

    for (unsigned int i=0; i<nb_ctr_; i++)
        constraints_[i]->update_dynamics(x,dyns_);
    parameterization_->compute(x,dyns_);

    int cpt = 0;
    for(int k=0;k<nb_robots_;k++)
    {
       visu_optim_->apply_q(robots_[k]->getRobotName(),&dyns_[k]->q_);
    }
   visu_optim_-> clear_lines();
    for (int i=0;i<constraints_.size();i++)
        constraints_[i]->update_visu(visu_optim_,dyns_,(const double*) x);
    if(show_result_)
       visu_optim_->wait_close();
#endif // MogsVisu_FOUND
}

void NLP_FAD_1_4::run_computation(const Number * x, unsigned int n, bool new_x)
{
	if (new_x)
	{
	    #ifdef PRINT
	    std::cout<<"new_x"<<std::endl;
	    #endif
		compute_number_ = true;
		compute_gradient_ = true;
	}
	if(compute_number_)
	{
       #ifdef PRINT
        std::cout<<"start computation of Number"<<std::endl;
        #endif // PRINT
		parameterization_->prepare_computation(dyns_);
		for (unsigned int i=0; i<nb_ctr_; i++)
			constraints_[i]->update_dynamics(x,dyns_);
		parameterization_->compute(x,dyns_);
		compute_number_ = false;
       #ifdef PRINT
        std::cout<<"end of computation of Number"<<std::endl;
        #endif // PRINT
	}
	run_gradient_computation(x,n,new_x);
}

void NLP_FAD_1_4::run_gradient_computation(const Number * x, unsigned int n, bool new_x)
{
	if (new_x)
	{
	    #ifdef PRINT
	    std::cout<<"new_x"<<std::endl;
	    #endif
		compute_number_ = true;
		compute_gradient_ = true;
	}
    if(compute_gradient_)
    {
       #ifdef PRINT
        std::cout<<"start computation of Gradient"<<std::endl;
        #endif // PRINT
        for(unsigned int i=0;i<n;i++)
        {
            X[i] = x[i];
            X[i].diff(i,n);
        }
        parameterization_->prepare_computation(adyns_);
        for (unsigned int i=0; i<nb_ctr_; i++)
            constraints_[i]->update_dynamics(X,adyns_);
        parameterization_->compute(X,adyns_);
        compute_gradient_ = false;
       #ifdef PRINT
        std::cout<<"end of computation of Gradient"<<std::endl;
        #endif // PRINT
    }
}

// For dynamic loading of the library : do not remove !!

void NLP_FAD_1_4::set_problem_properties(   const std::vector<MogsOptimDynamics<double>* >& dyns,
                                            AbstractParameterization* param,
                                            const std::vector<AbstractCriteria* > &criteres,
                                            const std::vector<AbstractConstraint*> & constraints)
{
    #ifdef PRINT
    std::cout<<"NLP_FAD_1_4::set_problem_properties"<<std::endl;
    #endif // PRINT

    nb_robots_ = robots_.size();

    for(int i=0;i<nb_robots_;i++)
    {
        dyns_.push_back(new MogsOptimDynamics<Number>(robots_[i]));
        adyns_.push_back(new MogsOptimDynamics<F<Number> >(robots_[i]));
    }
    AbstractLoader loader;

    parameterization_ =  dynamic_cast<AbstractFAD_1_4Parameterization*> (loader.get_parameterization<create_FAD_1_4Parameterization*>("MogsParameterizationNlpFAD_1_4",param));

    criteres_.clear();
    for (int i=0;i<criteres.size();i++)
    {
        AbstractFAD_1_4Critere* c = dynamic_cast<AbstractFAD_1_4Critere*> (loader.get_criteria<create_FAD_1_4Critere*>("MogsCriteriaNlpFAD_1_4",criteres[i]));
        criteres_.push_back(c);
    }

    constraints_.clear();
    for (int i=0;i<constraints.size();i++)
    {
        AbstractFAD_1_4Constraint* ctr = dynamic_cast<AbstractFAD_1_4Constraint*> (loader.get_constraint<create_FAD_1_4Constraint*>("MogsConstraintNlpFAD_1_4",constraints[i]));
        constraints_.push_back(ctr);
    }
}

extern "C" NLP_FAD_1_4* create()
{
    return new NLP_FAD_1_4();
}

extern "C" void destroy(NLP_FAD_1_4* p)
{
    delete p;
}


