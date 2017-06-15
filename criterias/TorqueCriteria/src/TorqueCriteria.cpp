#include <TorqueCriteria.hpp>

TorqueCriteria::TorqueCriteria (QDomElement critere,
								std::vector<MogsOptimDynamics<double> *>& dyns)
{
    weight_ = critere.attribute("weight").toDouble();

    std::cout<<"weight = "<< weight_<<std::endl;
	for (QDomElement ElRobot = critere.firstChildElement("robot"); !ElRobot.isNull(); ElRobot = ElRobot.nextSiblingElement("robot") )
	{
		QString robot_name = ElRobot.text().simplified();
		unsigned int nb = dyns.size();
		for (int i=0;i<nb;i++)
		{
			if( dyns[i]->getRobotName() == robot_name)
			{
				robot_id_.push_back(i);
				
				if(dyns[i]->model->is_robot_floating_base())
					start_.push_back(6);
				else
					start_.push_back(0);
				
				break;
			}
		}  		
	}
	nb_robots_ = robot_id_.size();
}

TorqueCriteria::~TorqueCriteria ()
{
}





