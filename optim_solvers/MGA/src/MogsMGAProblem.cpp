#include <MogsMGAProblem.h>

#include "AbstractDoubleCriteria.hpp"
#include "AbstractDoubleParameterization.hpp"
#include "AbstractDoubleConstraint.hpp"
#include "AbstractLoader.h"
#include "VisuHolder.h"

MogsMGAProblem::MogsMGAProblem ()
{
    std::cout<<"Construction of MogsMGAProblem"<<std::endl;
}

MogsMGAProblem::~MogsMGAProblem ()
{
}

void MogsMGAProblem::get_problem_info(unsigned int & nb_variables,
                                  unsigned int & nb_objectives,
                                  unsigned int & nb_constraints,
                                  std::vector<double>& seuils,
                                  std::vector<double>& min_var,
                                  std::vector<double>& max_var,
                                  std::vector<double>& min_ctr,
                                  std::vector<double>& max_ctr)
{
    unsigned int nb_ctr = constraints_.size();
    for(int i=0;i<nb_ctr;i++)
        parameterization_->init_from_constraints(constraints_[i]);
    nb_var_ = parameterization_->get_nb_param();
    nb_variables = nb_var_;
    min_var.resize(nb_variables);
    max_var.resize(nb_variables);

    for (int i=0;i<nb_var_;i++)
    {
        min_var[i] = parameterization_->get_bounds_inf(i);
        max_var[i] = parameterization_->get_bounds_sup(i);
    }

	nb_objectives = nb_crit_ = criteres_.size();
	min_ctr.clear();
	max_ctr.clear();
    for (int i=0;i<constraints_.size();i++)
    {
        constraints_[i]->set_offset(min_ctr.size());
        unsigned int n = constraints_[i]->get_nb_constraints();
        for (int j=0;j<n;j++)
        {
            min_ctr.push_back(constraints_[i]->get_lower(j));
            max_ctr.push_back(constraints_[i]->get_upper(j));
        }
    }
    nb_constraints = nb_ctr_ = min_ctr.size();
	for (int i=0;i<nb_crit_;i++)
        seuils.push_back(1e-3);

//	for (int i=0;i<nb_var_;i++)
//	{
//		std::cout<<"variables["<<i<<"] in ["<< min_var [i]<<":"<<max_var [i]<<":]"<<std::endl;
//	}
}

void MogsMGAProblem::evaluate(  std::vector<optim_infos> &infos)
{
    unsigned int N = constraints_.size();
    double x[nb_var_];
    double g [nb_ctr_];
    unsigned int cpt =0;

   	for (int j=0;j<infos.size();j++)    // for all the element of the generation
    {
        for (int i=0;i<nb_var_;i++)
            x[i] = infos[j].var[i];

        parameterization_->prepare_computation(dyns_);
        for (unsigned int i=0; i<N; i++)
            constraints_[i]->update_dynamics(x,dyns_);
        parameterization_->compute(x,dyns_);

		for (int i =0;i<nb_crit_;i++)
		{
			infos[j].obj[i] = criteres_[i]->compute(dyns_);
		}
		for (int i =0;i<N;i++)
		{
		    constraints_[i]->compute(x,g,dyns_);
		}

        for (int k=0;k<nb_ctr_;k++)
            infos[j].ctr[k] = g[k];
    }

    #ifdef MogsVisu_FOUND
    if (visu_during_optim_)
    {
        double objmax = 1e30;
        double ctrmax = 1e30;
        unsigned int mem=0;
        for (int j=0;j<infos.size();j++)
        {
            if (ctrmax > 1e-3)
            {
                if(ctrmax> infos[j].equivalent_ctr)
                {
                    ctrmax = infos[j].equivalent_ctr;
                    mem = j;
                }
            }else
            {
                if(infos[j].equivalent_ctr < 1e-3 && objmax> infos[j].obj[0])
                {
                    objmax = infos[j].obj[0];
                    mem = j;
                }
            }
        }

        for (int i=0;i<nb_var_;i++)
            x[i] = infos[mem].var[i];

        parameterization_->prepare_computation(dyns_);
        for (unsigned int i=0; i<N; i++)
            constraints_[i]->update_dynamics(x,dyns_);
        parameterization_->compute(x,dyns_);

        for(int k=0;k<nb_robots_;k++)
        {
            visu_optim_->apply_q(robots_[k]->getRobotName(),&dyns_[k]->q_);
        }

        visu_optim_-> clear_lines();
        for (int i=0;i<constraints_.size();i++)
            constraints_[i]->update_visu(visu_optim_,dyns_,(const double*) x);
    }
    #endif // MogsVisu_FOUND
}

void MogsMGAProblem::finalize_solution( optim_infos &info)
{
	std::cout<<"Optimal criteria = "<< info.obj[0]<<std::endl;
	printf("\n\nSolution of the variables, x\n");
	for (int i=0; i<nb_var_; i++) {
	printf("x[%d] = %e\n", i,  info.var[i]);
	}

#ifdef MogsVisu_FOUND
    unsigned int nb_ctr = constraints_.size();
    double x[nb_var_];
    for (int i=0;i<nb_var_;i++)
        x[i] = info.var[i];

    parameterization_->prepare_computation(dyns_);
    for (unsigned int i=0; i<nb_ctr; i++)
        constraints_[i]->update_dynamics(x,dyns_);
    parameterization_->compute(x,dyns_);

    for(int k=0;k<nb_robots_;k++)
    {
        visu_optim_->apply_q(robots_[k]->getRobotName(),&dyns_[k]->q_);
    }

    visu_optim_-> clear_lines();
    for (int i=0;i<constraints_.size();i++)
        constraints_[i]->update_visu(visu_optim_,dyns_,(const double*) x);
    visu_optim_->wait_close();
#endif // MogsVisu_FOUND
}

void MogsMGAProblem::load_xml( )
{
//    #ifdef PRINT
    std::cout<<"start load_xml"<<std::endl;
//    #endif // PRINT
    for (int i=0;i<nb_robots_;i++)
    {
        dyns_.push_back( new MogsOptimDynamics<double>(robots_[i]));
    }
    AbstractLoader loader;

	MogsProblemClassifier mpc;
	mogs_string library_so;
    QDomElement criteres=root_.firstChildElement("criteres");
    for (QDomElement critere = criteres.firstChildElement ("critere"); !critere.isNull();critere = critere.nextSiblingElement("critere"))
	{
        AbstractCriteria* crit = dynamic_cast<AbstractCriteria*> (loader.get_criteria<create_AbstractDoubleCriteria*>("MogsCriteriaDouble",critere,dyns_));
        criteres_.push_back(crit);
	}

    /// FIXME allow to change the type of the AbstractParameterization through plugins
    std::cout<<"nb_robot = "<< dyns_.size()<<std::endl;
    QDomElement param =root_.firstChildElement("parameterization");
    if (param.tagName()=="parameterization")
    {
        parameterization_ = dynamic_cast<AbstractParameterization*> (loader.get_parameterization<create_AbstractDoubleParameterization*>("MogsParameterizationDouble",param,dyns_));
    }else
    {
        std::cerr<<"ERROR cannot find balise parameterization"<<std::endl;
        exit(0);
    }

    QDomElement constraints=root_.firstChildElement("constraints");

    for (QDomElement constraint = constraints.firstChildElement ("constraint"); !constraint.isNull();constraint = constraint.nextSiblingElement("constraint"))
	{
        AbstractConstraint* ctr = dynamic_cast<AbstractConstraint*> (loader.get_constraint<create_AbstractDoubleConstraint*>("MogsConstraintDouble",constraint,dyns_));

         std::cout << "loading constraints name "   <<constraint.attribute("type").toStdString().c_str() << std::endl;
        constraints_.push_back(ctr);
	}

//    #ifdef PRINT
    std::cout<<"end of load_xml"<<std::endl;
//    #endif // PRINT
}
