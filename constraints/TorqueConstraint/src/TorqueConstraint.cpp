#include <TorqueConstraint.hpp>

TorqueConstraint::TorqueConstraint ()
{
    plugin_name_ = "Torque";
}

TorqueConstraint::~TorqueConstraint ()
{

}

void TorqueConstraint::init_from_xml( QDomElement ele,
                                       std::vector<MogsOptimDynamics<double> *>& dyns)
{
	m = 0;
	for (QDomElement ElRobot = ele.firstChildElement("robot"); !ElRobot.isNull(); ElRobot = ElRobot.nextSiblingElement("robot") )
	{
		QString name = ElRobot.text().simplified();
		unsigned int  nb = dyns.size();
		for (int i=0;i<nb;i++)
		{
			if( dyns[i]->getRobotName() == name)
			{
				robot_id_.push_back(i);
				std::vector < double >QMAX;
				dyns[i]->model->getTorqueLimit(QMAX);
				if( dyns[i]->model->is_robot_floating_base())
				{
				    start_.push_back(6);
					m+= dyns[i]->getNDof()-6;
					for (int ii=6;ii<dyns[i]->getNDof();ii++)
                    {
                        upper_.push_back(QMAX[ii]);
                        lower_.push_back(-QMAX[ii]);
                    }
				}
				else
				{
				    start_.push_back(0);
					m+= dyns[i]->getNDof();
					for (int ii=0;ii<dyns[i]->getNDof();ii++)
                    {
                        upper_.push_back(QMAX[ii]);
                        lower_.push_back(-QMAX[i]);
                    }
				}
                end_.push_back(dyns[i]->getNDof());
				break;
			}
		}
	}
}

void TorqueConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    *this =  *(dynamic_cast<TorqueConstraint*>(c));
}

#ifdef MogsVisu_FOUND
void TorqueConstraint::update_visu (VisuHolder *visu,
                          std::vector<MogsOptimDynamics<double> *> & dyns,
                          const double * param)
{

}
#endif // MogsVisu_FOUND
