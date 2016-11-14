#include <MogsNlpMGA.hpp>

#include "AbstractCriteria.hpp"
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

	nb_variables = kin_.getNDof();
	nb_objectives = 1;
	nb_constraints = 0;
	
	seuils.push_back(0);
	robots_[0]->getPositionLimit(min_var,max_var);
	
	if (kin_.is_free_floating_base())
	for (int i=0;i<3;i++)
	{
		if (min_var [i] < -1000.)	min_var [i] = -1000.;
		if (max_var [i] >  1000.)	max_var [i] =  1000.;
	}
	
	for (int i=0;i<nb_variables;i++)
	{
		
		std::cout<<"variables in ["<< min_var [i]<<":"<<max_var [i]<<":]"<<std::endl;
	}

}

void MogsNlpMGA::evaluate(  std::vector<optim_infos> &infos)
{
// 	std::cout<<"infos.size() = "<< infos.size() <<std::endl;
	
	double x[kin_.getNDof()];
	for (int j=0;j<infos.size();j++)
	{
		if (kin_.is_free_floating_base())
			for (int i=3;i<6;i++)
				infos[j].var[i] =	fmod(infos[j].var[i],2*Pi);
			
		for (int i =0;i<kin_.getNDof();i++)
			x[i] = infos[j].var[i];
		
		int nb = criteres_.size();
// 		std::cout<<"nb = "<<nb<<std::endl;
		double obj_value =0;
		bool mem_kin = false;
		for (int i =0;i<nb;i++)
		{
			double tmp = criteres_[i]->compute(x,&kin_,&mem_kin);
// 			std::cout<<"tmp("<<i<<") = "<<tmp<<std::endl;
			obj_value+= tmp;
		}
		
		infos[j].obj[0] = obj_value;
	}
	
}

void MogsNlpMGA::finalize_solution( optim_infos &info)
{
	double x[kin_.getNDof()];
	std::cout<<"Optimal criteria = "<< info.obj[0]<<std::endl;
	printf("\n\nSolution of the variables, x\n");
	for (int i=0; i<kin_.getNDof(); i++) {
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
    VisuHolder visu("resultats");

    visu.add("robot",robots_[0]);
	Eigen::Matrix < double,Eigen::Dynamic, 1 > q;
	q.resize(robots_[0]->getNDof());

    for (int i=0;i<robots_[0]->getNDof();i++)
        q(i) = info.var[i];
	std::cout<<"q = "<< q.transpose()<<std::endl;
	for (int i=0;i<3;i++)	q(i)= 0;
	
    visu.apply_q("robot",&q);

    visu.wait_close();
#endif // MogsVisu_FOUND
}

void MogsNlpMGA::load_xml(QDomElement criteres)
{
    

    double tval,weight_;
    QString type,weight,name;

    for (QDomElement critere = criteres.firstChildElement ("critere"); !critere.isNull();critere = critere.nextSiblingElement("critere"))
	{
		if (criteres.tagName()=="criteres")
		{
			qDebug()<<"Fix me , for the moment no criteria implemented !!";
/*			type=critere.attribute("type");
			name=critere.attribute("name");
			std::cout << "critere "   << type.toStdString().c_str() <<  std::endl;

			if(type=="position")
			{
				weight=critere.attribute("weight");
				std::istringstream smallData (weight.toStdString(), std::ios_base::in);
				smallData >> tval;
				weight_ = tval;
				std::cout << "   weight_ = " << weight_  << std::endl;
				//FIX ME
				criteres_.push_back(new PositionCriteria(critere,&kin_));

			}
			else if(type=="camera")
			{
			    //FIX ME
				std::cout << "name "   <<name.toStdString().c_str() << std::endl;
				criteres_.push_back(new CameraCriteria(critere,&kin_));
			}*/
		}
	}
}
void MogsNlpMGA::set_robots(const std::vector<MogsRobotProperties*> & in)
{
	robots_ = in;
	kin_.SetRobot(robots_[0]);
}
