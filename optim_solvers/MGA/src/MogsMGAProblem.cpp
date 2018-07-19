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
//

void MogsMGAProblem::get_problem_info(unsigned int & nb_variables,
                                  unsigned int & nb_objectives,
                                  unsigned int & nb_constraints,
                                  std::vector<double>& seuils,
                                  std::vector<double>& min_var,
                                  std::vector<double>& max_var,
                                  std::vector<double>& min_ctr,
                                  std::vector<double>& max_ctr)
{
    std::cout<<"get_problem_info debut "<<std::endl;
    unsigned int nb_ctr = constraints_.size();
    std::cout<<"nb_ctr = "<< nb_ctr <<std::endl;
    for(int i=0;i<nb_ctr;i++)
        parameterization_->init_from_constraints(constraints_[i]);
    std::cout<<"init from constraint done"<<std::endl;
    nb_var_ = parameterization_->get_nb_param();
    nb_variables = nb_var_;
    std::cout<<"nb_variables = "<< nb_variables <<std::endl;
    min_var.resize(nb_variables);
    max_var.resize(nb_variables);

    for (int i=0;i<nb_var_;i++)
    {
        min_var[i] = parameterization_->get_bounds_inf(i);
        max_var[i] = parameterization_->get_bounds_sup(i);
    }
    std::cout<<"prepare bounds"<<std::endl;
//    nb_variables=0;
//    for(int i=0;i<nb_robots_;i++)
//        nb_variables += dyns_[i]->getNDof();
//
	nb_objectives = nb_crit_ = criteres_.size();

	min_ctr.clear();
	max_ctr.clear();
    std::cout<<"constraints_.size() = "<< constraints_.size()<<std::endl;
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
    std::cout<<"nb_constraints = "<< nb_constraints <<std::endl;
	for (int i=0;i<nb_crit_;i++)
        seuils.push_back(1e-3);

//
//	for (unsigned int i = 0; i<nb_robots_;i++)
//    {
//        std::vector<double> minq,maxq;
//        robots_[0]->getPositionLimit(minq,maxq);
//        if (dyns_[0]->is_free_floating_base())
//            for (int i=0;i<3;i++)
//            {
//                if (minq [i] < -1000.)	minq [i] = -1000.;
//                if (maxq [i] >  1000.)	maxq [i] =  1000.;
//            }
//        for (unsigned int j=0;j<minq.size();j++)
//        {
//            min_var.push_back(minq[j]);
//            max_var.push_back(maxq[j]);
//        }
//    }

	for (int i=0;i<nb_var_;i++)
	{
		std::cout<<"variables["<<i<<"] in ["<< min_var [i]<<":"<<max_var [i]<<":]"<<std::endl;
	}
    std::cout<<"get_problem_info fin "<<std::endl;
}

void MogsMGAProblem::evaluate(  std::vector<optim_infos> &infos)
{
// 	std::cout<<"infos.size() = "<< infos.size() <<std::endl;
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



//    double x[nb_variables];
//    unsigned int cpt =0;
//   	for (int j=0;j<infos.size();j++)
//    {
//        x[cpt++] = infos[j].var[i];
//    }
//
//
//    for (unsigned int i=0; i<nb_ctr_; i++)
//        constraints_[i]->update_dynamics(x,adyns_);
//    parameterization_->compute(x,adyns_);
//
//	for (int j=0;j<infos.size();j++)
//	{
//		if (dyns_[0]->is_free_floating_base())
//			for (int i=3;i<6;i++)
//				infos[j].var[i] =	fmod(infos[j].var[i],2*Pi);
//
//		for (int i =0;i<dyns_[0]->getNDof();i++)
//			x[i] = infos[j].var[i];
//
//		int nb = criteres_.size();
//		double obj_value =0;
//		bool mem_kin = false;
//		for (int i =0;i<nb;i++)
//		{
//			double tmp = criteres_[i]->compute(x,dyns_,&mem_kin);
//			obj_value+= tmp;
//		}
//
//		infos[j].obj[0] = obj_value;
//	}

}

void MogsMGAProblem::finalize_solution( optim_infos &info)
{
	std::cout<<"Optimal criteria = "<< info.obj[0]<<std::endl;
	printf("\n\nSolution of the variables, x\n");
	for (int i=0; i<nb_var_; i++) {
	printf("x[%d] = %e\n", i,  info.var[i]);
	}


//	 std::vector<optim_infos> tmp;
//	 tmp.push_back(info);
//	 evaluate(tmp);
//	 std::cout<<"Optimal criteria = "<< tmp[0].obj[0]<<std::endl;
//	 std::cout<<"tmp criteria = "<< tmp[1].obj[0]<<std::endl;

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

//void MogsMGAProblem::set_problem_properties(   const std::vector<MogsOptimDynamics<double>* >& dyns,
//                                            AbstractParameterization* param,
//                                            const std::vector<AbstractCriteria* > &criteres,
//                                            const std::vector<AbstractConstraint*> & constraints)
//{
//    std::cout<<"set problem properties" <<std::endl;
//    #ifdef PRINT
//    std::cout<<"NLP_Double::set_problem_properties"<<std::endl;
//    #endif // PRINT
//
//    nb_robots_ = robots_.size();
//
//    for(int i=0;i<nb_robots_;i++)
//    {
//        dyns_.push_back(new MogsOptimDynamics<double>(robots_[i]));
//    }
//    AbstractLoader loader;
//
//    parameterization_ =  dynamic_cast<AbstractParameterization*> (loader.get_parameterization<create_Parameterization*>("MogsParameterization",param));
//
//    criteres_.clear();
//    for (int i=0;i<criteres.size();i++)
//    {
//        AbstractCriteria* c = dynamic_cast<AbstractCriteria*> (loader.get_criteria<create_Criteria*>("MogsCriteria",criteres[i]));
//        criteres_.push_back(c);
//    }
//
//    constraints_.clear();
//    for (int i=0;i<constraints.size();i++)
//    {
//        AbstractConstraint* ctr = dynamic_cast<AbstractConstraint*> (loader.get_constraint<create_Constraint*>("MogsConstraint",constraints[i]));
//        constraints_.push_back(ctr);
//    }
//}

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

//void MogsMGAProblem::set_problem_properties(   const std::vector<MogsOptimDynamics<double>* >& dyns,
//                                            AbstractParameterization* param,
//                                            const std::vector<AbstractCriteria* > &criteres,
//                                            const std::vector<AbstractConstraint*> & constraints)
//{
//    std::cout<<"set problem properties" <<std::endl;
//    #ifdef PRINT
//    std::cout<<"NLP_Double::set_problem_properties"<<std::endl;
//    #endif // PRINT
//
//    nb_robots_ = robots_.size();
//
//    for(int i=0;i<nb_robots_;i++)
//    {
//        dyns_.push_back(new MogsOptimDynamics<double>(robots_[i]));
//    }
//    AbstractLoader loader;
//
//    parameterization_ =  dynamic_cast<AbstractParameterization*> (loader.get_parameterization<create_Parameterization*>("MogsParameterization",param));
//
//    criteres_.clear();
//    for (int i=0;i<criteres.size();i++)
//    {
//        AbstractCriteria* c = dynamic_cast<AbstractCriteria*> (loader.get_criteria<create_Criteria*>("MogsCriteria",criteres[i]));
//        criteres_.push_back(c);
//    }
//
//    constraints_.clear();
//    for (int i=0;i<constraints.size();i++)
//    {
//        AbstractConstraint* ctr = dynamic_cast<AbstractConstraint*> (loader.get_constraint<create_Constraint*>("MogsConstraint",constraints[i]));
//        constraints_.push_back(ctr);
//    }
//}

//void MogsMGAProblem::set_problem_properties(   const std::vector<MogsOptimDynamics<double>* >& dyns,
//                                            AbstractParameterization* param,
//                                            const std::vector<AbstractCriteria* > &criteres,
//                                            const std::vector<AbstractConstraint*> & constraints)
//{
//    #ifdef PRINT
//    std::cout<<"MogsMGAProblem::set_problem_properties"<<std::endl;
//    #endif // PRINT
//
//    nb_robots_ = robots_.size();
//
//    for(int i=0;i<nb_robots_;i++)
//    {
//        dyns_.push_back(new MogsOptimDynamics<double>(robots_[i]));
//    }
//    AbstractLoader loader;
//
//    parameterization_ =  dynamic_cast<AbstractDoubleParameterization*> (loader.get_parameterization<create_AbstractDoubleParameterization*>("MogsParameterization",param));
//
//    criteres_.clear();
//    for (int i=0;i<criteres.size();i++)
//    {
//        AbstractCriteria* c = dynamic_cast<AbstractDoubleCriteria*> (loader.get_criteria<create_AbstractDoubleCriteria*>("MogsCriteria",criteres[i]));
//        criteres_.push_back(c);
//    }
//
//    constraints_.clear();
//    for (int i=0;i<constraints.size();i++)
//    {
//        AbstractConstraint* ctr = dynamic_cast<AbstractDoubleConstraint*> (loader.get_constraint<create_AbstractDoubleConstraint*>("MogsConstraint",constraints[i]));
//        constraints_.push_back(ctr);
//    }
//}
//
//void MogsMGAProblem::set_robots(const std::vector<MogsRobotProperties*> & in)
//{
//	robots_ = in;
//	nb_robots_ = robots_.size();
//}
