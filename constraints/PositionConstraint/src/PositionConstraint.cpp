#include <PositionConstraint.hpp>

PositionConstraint::PositionConstraint (  )
{
    plugin_name_ = "Position";
}

PositionConstraint::PositionConstraint(  std::vector<MogsOptimDynamics<double> *> &dyns,
                                         const QString& robot_name,
                                         const QString& body_name,
                                         const Eigen::Matrix<double,3,1>& body_position,
                                         const Eigen::Matrix<double,3,1>& desired_position):PositionConstraint()
{
    Robot = robot_name;
    robot_id_ = -1;
    for (int i=0;i<dyns.size();i++)
    {
        if(Robot == dyns[i]->getRobotName())
        {
            robot_id_ = i;
            break;
        }
    }
    if(robot_id_==-1)
    {
        std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
        std::cerr<<"Error cannot recognize robot_id !!"<<std::endl;
        exit(-1);
    }

    Body=body_name;
    body_id_ = dyns[robot_id_]->model->GetBodyId(Body);
    if (body_id_  ==  std::numeric_limits <unsigned int >::max () )
    {
        std::cout << "   Body_ (" <<  Body.toStdString() <<") is unkown"<< std::endl;
        exit(0);
    }
    body_Position_ = body_position;
    desired_Position_ = desired_position;

    m = 3; //desired_Position_.size();
    upper_.resize(3);
    lower_.resize(3);
    for (int i=0;i<3;i++)
    {
        upper_[i] = lower_[i] = desired_Position_(i);
    }

}

void PositionConstraint::init_from_xml(   QDomElement contraint,
                                            std::vector<MogsOptimDynamics<double> *>& dyns)
{
    qDebug()<<"Constructor of PositionConstraint";
    m = 3; //desired_Position_.size();
    upper_.resize(3);
    lower_.resize(3);
    desired_Position_ = Eigen::Matrix<double,3,1>::Zero();
    QDomElement Child=contraint.firstChildElement().toElement();

    while (!Child.isNull())
             {
                 if (Child.tagName()=="robot")
                   {
                        Robot=Child.firstChild().toText().data().simplified();
                        std::cout << "   Robot  = " << Robot.toStdString().c_str() << std::endl;
                        robot_id_ = -1;
                        for (int i=0;i<dyns.size();i++)
                            if(Robot == dyns[i]->getRobotName())
                            {
                                robot_id_ = i;
                                break;
                            }
                        if(robot_id_==-1)
                        {
                            std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
                            std::cerr<<"Error cannot recognize robot_id !!"<<std::endl;
                            exit(-1);
                        }
                   }
                 if (Child.tagName()=="body")
                   {
                                Body=Child.firstChild().toText().data().simplified();
                                std::cout << "   Body  = " << Body.toStdString() << std::endl;
                                body_id_ = dyns[robot_id_]->model->GetBodyId(Body);
                         if (body_id_  ==  std::numeric_limits <unsigned int >::max () )
                            {
                                  std::cout << "   Body_ (" <<  Body.toStdString() <<") is unkown"<< std::endl;
                                  exit(0);
                            }
                                  std::cout << "   body_id_  = " << body_id_ << std::endl;
                    }
                if (Child.tagName()=="body_position")
                    {
                        body_Position = Child.attribute("body_position");
                        body_Position=Child.firstChild().toText().data();
                        std::istringstream smallData (body_Position.toStdString(), std::ios_base::in);
                           for (int i = 0; i < 3; i++)
                                  {
                                    smallData >> tval;
                                    body_Position_(i) = tval;
                                  }
                         std::cout << "   body_Position  = " <<    body_Position_  << std::endl;
                    }
                if (Child.tagName()=="desired_position")
                {
                    desired_Position = Child.attribute("desired_position");
                    desired_Position=Child.firstChild().toText().data();
                    std::istringstream smallData (desired_Position.toStdString(), std::ios_base::in);
                    for (int i = 0; i < 3; i++)
                    {
                        smallData >> tval;
                        desired_Position_(i) = tval;
                    }

                }
                else
                {
                    std::cerr<<"ERROR you did not specify desired position"<<std::endl;
                    std::cerr<<"Considering zero."<<std::endl;
                }

               Child = Child.nextSibling().toElement();
            }
    std::cout<<"desired_Position_ = "<< desired_Position_.transpose() <<std::endl;

    for (int i=0;i<3;i++)
    {
        upper_[i] = lower_[i] = desired_Position_(i);
    }
}
   //

PositionConstraint::~PositionConstraint ()
{
}


void PositionConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    std::cout<<"c->m = "<< c->get_nb_constraints() <<std::endl;
    *this =  *(dynamic_cast<PositionConstraint*>(c));
    std::cout<<"m = "<< m <<std::endl;
}
