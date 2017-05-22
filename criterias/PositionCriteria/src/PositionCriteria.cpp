#include <PositionCriteria.hpp>


PositionCriteria::PositionCriteria (QDomElement critere,
                                     std::vector<MogsOptimDynamics<double> *> dyns)
{
    weight_ = critere.attribute("weight").toDouble();
//    std::cout<<"weight = "<< weight_<<std::endl;
    QDomElement Child=critere.firstChildElement().toElement();
    while (!Child.isNull())
    {
        if (Child.tagName()=="robot")
        {
            Robot=Child.firstChild().toText().data().simplified();
//            qDebug() << "   Robot  = " << Robot;
            robot_id_ = -1;
            for (int i=0;i<dyns.size();i++)
            {
//                qDebug()<<"robot name = "<< dyns[i]->getRobotName();
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
        }
        if (Child.tagName()=="body")
        {
            Body=Child.firstChild().toText().data().simplified();
//            std::cout << "   Body  = " << Body.toStdString() << std::endl;
            body_id_ = dyns[robot_id_]->model->GetBodyId(Body);
            if (body_id_  ==  std::numeric_limits <unsigned int >::max () )
            {
                std::cout << "   Body_ (" <<  Body.toStdString() <<") is unkown"<< std::endl;
                exit(0);
            }
//            std::cout << "   body_id_  = " << body_id_ << std::endl;
        }
        if (Child.tagName()=="body_position")
        {
            body_position = Child.attribute("body_position");
            body_position=Child.firstChild().toText().data();
            std::istringstream smallData (body_position.toStdString(), std::ios_base::in);
            for (int i = 0; i < 3; i++)
            {
                smallData >> tval;
                body_position_(i) = tval;
            }
//            std::cout << "   body_position  = " <<    body_position_  << std::endl;
        }
        if (Child.tagName()=="desired_position")
        {
            desired_position = Child.attribute("desired_position");
            desired_position=Child.firstChild().toText().data();
            std::istringstream smallData (desired_position.toStdString(), std::ios_base::in);
            for (int i = 0; i < 3; i++)
            {
                smallData >> tval;
                desired_position_(i) = tval;
            }
//            std::cout << "   desired_position  = " << desired_position_  << std::endl;
            }
            Child = Child.nextSibling().toElement();
    }
}

PositionCriteria::~PositionCriteria ()
{
}





