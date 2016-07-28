#include <PositionAdolcCritere.hpp>


PositionAdolcCritere::PositionAdolcCritere (QDomElement critere,
                                            MogsKinematics<double>* kin)
{
    body_id_=7;

    QString Body;
    QString Robot;
    QString body_position;
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
                Body=Child.firstChild().toText().data();

                    MogsKinematics<double>* model;
                    body_id_ = kin->GetBodyId(&model);

                std::cout << "   Body  = " << Body.toStdString().c_str() << std::endl;
                }
                if (Child.tagName()=="body_position")
                {body_position=Child.firstChild().toText().data();
                std::cout << "   body_position  = " << body_position.toStdString().c_str() << std::endl;
                }
                 if (Child.tagName()=="desired_position")
                {desired_position=Child.firstChild().toText().data();
                 std::cout << "   desired_position  = " << desired_position.toStdString().c_str() << std::endl;
                }
               Child = Child.nextSibling().toElement();
            }
}

PositionAdolcCritere::~PositionAdolcCritere ()
{
}





