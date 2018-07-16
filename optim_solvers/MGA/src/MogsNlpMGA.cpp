#include <MogsNlpMGA.hpp>

#include "AbstractCriteria.h"
#include "AbstractLoader.h"
#include "VisuHolder.h"


MogsNlpMGA::MogsNlpMGA ()
{
    std::cout<<"Construction of MogsNlpMGA"<<std::endl;
}

MogsNlpMGA::~MogsNlpMGA ()
{
}
//

void MogsNlpMGA::get_problem_info(unsigned int & nb_variables,
                                  unsigned int & nb_objectives,
                                  unsigned int & nb_constraints,
                                  std::vector<double>& seuils,
                                  std::vector<double>& min_var,
                                  std::vector<double>& max_var)
{
    if (nb_robots_ != 1)
    {
        std::cerr<<"Error for the moment MogsNLPMGA is only for one robot"<<std::endl;
        exit(0);
    }

    nb_variables=0;
    for(int i=0;i<nb_robots_;i++)
        nb_variables += dyns_[i]->getNDof();

	nb_objectives = 1;
	nb_constraints = 0;
	seuils.push_back(0);

	for (unsigned int i = 0; i<nb_robots_;i++)
    {
        std::vector<double> minq,maxq;
        robots_[0]->getPositionLimit(minq,maxq);
        if (dyns_[0]->is_free_floating_base())
            for (int i=0;i<3;i++)
            {
                if (minq [i] < -1000.)	minq [i] = -1000.;
                if (maxq [i] >  1000.)	maxq [i] =  1000.;
            }
        for (unsigned int j=0;j<minq.size();j++)
        {
            min_var.push_back(minq[j]);
            max_var.push_back(maxq[j]);
        }
    }

	for (int i=0;i<nb_variables;i++)
	{
		std::cout<<"variables in ["<< min_var [i]<<":"<<max_var [i]<<":]"<<std::endl;
	}

}

void MogsNlpMGA::evaluate(  std::vector<optim_infos> &infos)
{
// 	std::cout<<"infos.size() = "<< infos.size() <<std::endl;

	double x[dyns_[0]->getNDof()];
	for (int j=0;j<infos.size();j++)
	{
		if (dyns_[0]->is_free_floating_base())
			for (int i=3;i<6;i++)
				infos[j].var[i] =	fmod(infos[j].var[i],2*Pi);

		for (int i =0;i<dyns_[0]->getNDof();i++)
			x[i] = infos[j].var[i];

		int nb = criteres_.size();
		double obj_value =0;
		bool mem_kin = false;
		for (int i =0;i<nb;i++)
		{
		    std::cout<<__FILE__<<" at line "<< __LINE__<<" must be set properly"<<std::endl;
//			double tmp = criteres_[i]->compute(x,dyns_,&mem_kin);
//			obj_value+= tmp;
		}

		infos[j].obj[0] = obj_value;
	}

}

void MogsNlpMGA::finalize_solution( optim_infos &info)
{
	double x[dyns_[0]->getNDof()];
	std::cout<<"Optimal criteria = "<< info.obj[0]<<std::endl;
	printf("\n\nSolution of the variables, x\n");
	for (int i=0; i<dyns_[0]->getNDof(); i++) {
	printf("x[%d] = %e\n", i,  info.var[i]);
	}


	 std::vector<optim_infos> tmp;
	 optim_infos t = info;
	 t.var[6] = 0.;
	 tmp.push_back(info);
	 tmp.push_back(t);
	 evaluate(tmp);
	 std::cout<<"Optimal criteria = "<< tmp[0].obj[0]<<std::endl;
	 std::cout<<"tmp criteria = "<< tmp[1].obj[0]<<std::endl;

#ifdef MogsVisu_FOUND
    Eigen::Matrix<double,Eigen::Dynamic,1> q(robots_[0]->getNDof());
    for (int i=0;i<robots_[0]->getNDof();i++)
        q(i) = info.var[i];
	std::cout<<"q = "<< q.transpose()<<std::endl;
	for (int i=0;i<3;i++)	q(i)= 0;
    visu_optim_->apply_q(robots_[0]->getRobotName(),&q);
    visu_optim_->wait_close();
#endif // MogsVisu_FOUND
}

//void MogsNlpMGA::set_problem_properties(   const std::vector<MogsOptimDynamics<double>* >& dyns,
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

void MogsNlpMGA::load_xml( )
{
    #ifdef PRINT
    std::cout<<"start load_xml"<<std::endl;
    #endif // PRINT
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
        AbstractCriteria* crit = dynamic_cast<AbstractCriteria*> (loader.get_criteria<create_Criteria*>("MogsCriteria",critere,dyns_));
        criteres_.push_back(crit);
	}

    /// FIXME allow to change the type of the AbstractParameterization through plugins
    QDomElement param =root_.firstChildElement("parameterization");
    if (param.tagName()=="parameterization")
    {
        parameterization_ = dynamic_cast<AbstractParameterization*> (loader.get_parameterization<create_Parameterization*>("MogsParameterization",param,dyns_));
    }else
    {
        std::cerr<<"ERROR cannot find balise parameterization"<<std::endl;
        exit(0);
    }

    QDomElement constraints=root_.firstChildElement("constraints");

    for (QDomElement constraint = constraints.firstChildElement ("constraint"); !constraint.isNull();constraint = constraint.nextSiblingElement("constraint"))
	{
        AbstractConstraint* ctr = dynamic_cast<AbstractConstraint*> (loader.get_constraint<create_Constraint*>("MogsConstraint",constraint,dyns_));

        // std::cout << "loading constraints name "   <<constraint.attribute("type").toStdString().c_str() << std::endl;
        constraints_.push_back(ctr);
	}

    #ifdef PRINT
    std::cout<<"end of load_xml"<<std::endl;
    #endif // PRINT
}

//void MogsNlpMGA::set_problem_properties(   const std::vector<MogsOptimDynamics<double>* >& dyns,
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


void MogsNlpMGA::set_robots(const std::vector<MogsRobotProperties*> & in)
{
	robots_ = in;
	nb_robots_ = robots_.size();
}
