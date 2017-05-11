#include <PositionDConstraint.hpp>

PositionDConstraint::PositionDConstraint (  QDomElement contraint,
                                                    MogsKinematics<double>* kin)
{
    qDebug()<<"Constructor of PositionDConstraint";
    //m = 1;
    n = kin->getNDof();
    m = 3; //desired_Position_.size();
    std::cout << " m  = " << m << std::endl;
    //Eigen::Matrix<double, 3, 1> upper (0.4 , 0.4 , 0.3);
    upper.resize(3);
    lower.resize(3);
    desired_Position_ = Eigen::Matrix<double,3,1>::Zero();
    //test= 25;
    //Eigen::Matrix<double, 3, 1> lower (0.4 , 0.4 , 0.3);
    //std::cout << " test  = " << test << std::endl;

//    weight_ = critere.attribute("weight").toDouble();
//    std::cout<<"weight = "<< weight_<<std::endl;
    QDomElement Child=contraint.firstChildElement().toElement();

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
                if (Child.tagName()=="body_Position")
                    {
                        body_Position = Child.attribute("body_Position");
                        body_Position=Child.firstChild().toText().data();
                        std::istringstream smallData (body_Position.toStdString(), std::ios_base::in);
                           for (int i = 0; i < 3; i++)
                                  {
                                    smallData >> tval;
                                    body_Position_(i) = tval;
                                  }
                         std::cout << "   body_Position  = " <<    body_Position_  << std::endl;
                    }
                 if (Child.tagName()=="desired_Position")
                    {
                         desired_Position = Child.attribute("desired_Position");
                         desired_Position=Child.firstChild().toText().data();
                         std::istringstream smallData (desired_Position.toStdString(), std::ios_base::in);
                            for (int i = 0; i < 3; i++)
                                    {
                                        smallData >> tval;
                                        desired_Position_(i) = tval;
                                    }
                         std::cout << "   desired_Position  = " << desired_Position_.transpose()  << std::endl;
                    }
               Child = Child.nextSibling().toElement();
            }
    upper = desired_Position_;
    lower = desired_Position_;
}
   //

PositionDConstraint::~PositionDConstraint ()
{
}
