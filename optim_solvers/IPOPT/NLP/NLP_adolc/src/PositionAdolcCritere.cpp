#include <PositionAdolcCritere.hpp>


PositionAdolcCritere::PositionAdolcCritere (QDomElement critere,
                                            MogsKinematics<double>* kin)
{
    QString Body;
    QString Robot;
    QString body_position;
    body_position_(0) = 0.0;
	body_position_(1) = 0.0;
	body_position_(2) = 0.0;
    desired_position_(0) = 0.0;
	desired_position_(1) = 0.0;
	desired_position_(2) = 0.0;
    QString desired_position;

     QDomElement Child=critere.firstChildElement().toElement();
    while (!Child.isNull())
            {
                  if (Child.tagName()=="robot")
                {
                Robot=Child.firstChild().toText().data();
                std::cout << "   Robot  = " << Robot.toStdString().c_str() << std::endl;

                }
                 if (Child.tagName()=="body")
                {
                Body=Child.firstChild().toText().data().simplified();
                  std::cout << "   Body  = " << Body.toStdString() << std::endl;
                 body_id_ = kin->model->GetBodyId(Body);
                 if (body_id_  ==  std::numeric_limits <unsigned int >::max () )
                 {
                      std::cout << "   Body_ (" <<  Body.toStdString() <<") is unkown"<< std::endl;
                      exit(0);
                 }
                  std::cout << "   body_id_  = " << body_id_ << std::endl;

                }
                if (Child.tagName()=="body_position")
                    {
                        body_position = Child.attribute("body_position");
                        body_position=Child.firstChild().toText().data();
                        double tval;
                        std::istringstream smallData (body_position.toStdString(), std::ios_base::in);
                        for (int i = 0; i < 3; i++)
                                {
                                    smallData >> tval;
                                    body_position_(i) = tval;
                                }
                         std::cout << "   body_position  = " <<    body_position_  << std::endl;
                    }
                 if (Child.tagName()=="desired_position")
                    {
                         desired_position = Child.attribute("desired_position");
                         desired_position=Child.firstChild().toText().data();
                         double tval;
                            std::istringstream smallData (desired_position.toStdString(), std::ios_base::in);
                            for (int i = 0; i < 3; i++)
                                    {
                                        smallData >> tval;
                                        desired_position_(i) = tval;
                                    }
                         std::cout << "   desired_position  = " << desired_position_  << std::endl;
                    }
               Child = Child.nextSibling().toElement();
            }
}

PositionAdolcCritere::~PositionAdolcCritere ()
{
}





