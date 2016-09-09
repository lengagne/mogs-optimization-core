#include <MogsNlpMGA.hpp>

MogsNlpMGA::MogsNlpMGA ()
{
    std::cout<<"Construction of MogsNlpMGA"<<std::endl;
}

MogsNlpMGA::~MogsNlpMGA ()
{
}
////
//void MogsNlpMGA::set_robot__url(const std::vector<mogs_string> & in)
//{
//    robot_s_url_ = in;
//    robot_.Setrobot_File(robot_s_url_[0]);
//
//
//}


void MogsNlpMGA::get_problem_info(unsigned int & nb_variables,
                                  unsigned int & nb_objectives,
                                  unsigned int & nb_constraints,
                                  std::vector<double>& seuils,
                                  std::vector<double>& min_var,
                                  std::vector<double>& max_var)
{

}

void MogsNlpMGA::evaluate(  std::vector<optim_infos> &infos)
{

}

void MogsNlpMGA::finalize_solution( optim_infos &info)
{

}

void MogsNlpMGA::load_xml(QDomElement criteres)
{
    kin_.SetRobot(&robot_);

    double tval,weight_;
    QString type,weight,name;

    for (QDomElement critere = criteres.firstChildElement ("critere"); !critere.isNull();critere = critere.nextSiblingElement("critere"))
	{

		if (criteres.tagName()=="criteres")
		{

			type=critere.attribute("type");
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
//				criteres_.push_back(new PositionAdolcCritere(critere,&kin_));

			}
			else if(type=="camera")
			{
			    //FIX ME
				std::cout << "name "   <<name.toStdString().c_str() << std::endl;
//				criteres_.push_back(new CameraAdolcCritere(critere,&kin_));
			}
		}
	}
}


