#include <CloseToMiddleCriteria.hpp>

CloseToMiddleCriteria::CloseToMiddleCriteria (  QDomElement critere,
                                                std::vector<MogsOptimDynamics<double> *>& dyns)
{
    weight_ = critere.attribute("weight").toDouble();

    std::cout<<"weight = "<< weight_<<std::endl;
    QDomElement Child=critere.firstChildElement("robot");
    if (!Child.isNull())
	{
		Robot=Child.text().simplified();
		std::cout << "   Robot  = " << Robot.toStdString().c_str() << std::endl;
	}
    else
	{
		std::cerr<<"Error no balise robot found for CloseToMiddleCriteria"<<std::endl;
		exit(1);
    }
    
    robot_id_ = -1;
    unsigned int nb = dyns.size();
    for (int i=0;i<nb;i++)
    {
        if( dyns[i]->getRobotName() == Robot)
        {
            robot_id_ = i;
            break;
        }
    }    

   // calcul de qm
    // a partir de dyns[0]->model
    std::vector<double> qmin;
    std::vector<double> qmax;
    dyns[robot_id_]->model->getPositionLimit(qmin,qmax);
    qm_.resize(dyns[robot_id_]->getNDof());
    for (int i=0; i<dyns[robot_id_]->getNDof(); i++)
    {
        qm_(i) = (qmin[i] + qmax[i])/2;
    }


}

CloseToMiddleCriteria::~CloseToMiddleCriteria ()
{
}





