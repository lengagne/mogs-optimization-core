#include <CameraAdolcCritere.hpp>


 CameraAdolcCritere::CameraAdolcCritere (QDomElement critere,
                                            MogsKinematics<double>* kin)

{
    QDomElement Child=critere.firstChildElement().toElement();
    QDomElement point= critere.firstChildElement ("point");
    QDomElement h=point.firstChildElement().toElement();
         nb_points_=0;


         while (!point.isNull())
                            {
                                 if (point.tagName()=="point")

{                           nb_points_++;
                            std::cout << "   nb_points_=  " << nb_points_ << std::endl;

}
       while (!h.isNull())
                            {
                                 if (h.tagName()=="robot")
                                        {
                                     robot = h.attribute("robot");
                                     robot= h.firstChild().toText().data();
                                    std::cout << "   Robot  = " << robot.toStdString().c_str() << std::endl;
                                        }
                                 if (h.tagName()=="Body")
                                     {
                                     Body = h.attribute("Body");
                                    Body=h.firstChild().toText().data().simplified();
                                    std::cout << "   Body  = " << Body.toStdString() << std::endl;
                                    body_id_ = kin->model->GetBodyId(Body);
                                if (body_id_  ==  std::numeric_limits <unsigned int >::max () )
                                      {
                                      std::cout << "   Body_ (" <<  Body.toStdString() <<") is unkown"<< std::endl;
                                      exit(0);
                                        }
                                            std::cout << "   body_id_  = " << body_id_ << std::endl;
                                       }

                                if (h.tagName()=="body_position")
                                        {
                                            body_position = h.attribute("body_position");
                                            body_position=h.firstChild().toText().data();
                                            std::istringstream smallData (body_position.toStdString(), std::ios_base::in);
                                               for (int i = 0; i < 3; i++)
                                                      {
                                                        smallData >> tval;
                                                        body_position_(i) = tval;
                                                      }
                                             std::cout << "   body_position  = " <<    body_position_  << std::endl;
                                        }
                                        if (h.tagName()=="desired_position_image")
                                        {
                                            desired_position_image = h.attribute("desired_position_image");
                                            desired_position_image=h.firstChild().toText().data();
                                            std::istringstream smallData (desired_position_image.toStdString(), std::ios_base::in);
                                               for (int i = 0; i < 2; i++)
                                                      {
                                                        smallData >> tval;
                                                        desired_position_image_(i) = tval;
                                                      }
                                             std::cout << "   desired_position_image_  = " <<    desired_position_image_  << std::endl;
                                        }
                                                   h= h.nextSibling().toElement();

                                  }
point= point.nextSibling().toElement();

}
         while (!Child.isNull())
             {

                   if (Child.tagName()=="focale")
                   {
                        focale=Child.firstChild().toText().data().toDouble();std::cout << "   focale = " << focale << std::endl;
                   }
                   if (Child.tagName()=="alpha_u")
                   {  int i=0;
                            alpha_u=Child.firstChild().toText().data().toDouble();
                            std::cout << "   alpha_u = " << alpha_u  << std::endl;
                            i=i++;std::cout << "   i = " <<i  << std::endl;
                   }
                    if (Child.tagName()=="alpha_v")
                   {
                            alpha_v=Child.firstChild().toText().data().toDouble();
                            std::cout << "   alpha_v = " << alpha_v  << std::endl;
                        }

                    if (Child.tagName()=="u0")    {

                            u0=Child.firstChild().toText().data().toDouble();
                            std::cout << "   u0 = " << u0  << std::endl;

                   }
                    if (Child.tagName()=="v0")
                   {
                            v0=Child.firstChild().toText().data().toDouble();
                            std::cout << "   v0 = " << v0  << std::endl;

                   }
                   if (Child.tagName()=="Position")
                   {
                            Position=critere.attribute("Position");
                            Position=Child.firstChild().toText().data();
                            std::istringstream smallData (Position.toStdString(), std::ios_base::in);
                               for (int i = 0; i < 3; i++)
                                    {
                                        smallData >> tval;
                                        Position_(i) = tval;
                                    }
                            std::cout << "   Position_ = " << Position_  << std::endl;
                   }
                     if (Child.tagName()=="RotationX")
                   {
                            RotationX=Child.firstChild().toText().data().toDouble();
                            std::cout << "  RotationX = " << RotationX  << std::endl;

                                R << 1, 0, 0,
                                     0, cos(RotationX), -sin(RotationX),
                                     0, sin(RotationX), cos(RotationX);
                                     std::cout << "  R est égal = " << R <<std::endl;

                      }
                        Child = Child.nextSibling().toElement();
                }


                                    RT << 1,          0,         0,    Position_(0),
                                          0, cos(RotationX), -sin(RotationX),Position_(1),
                                          0, sin(RotationX), cos(RotationX),Position_(2);
                                         std::cout << "  RT est égal = " << RT <<std::endl;
                                    K << alpha_u ,  0,     u0,
                                          0,      alpha_v ,v0,
                                          0,        0,     1;
                                       std::cout << "  K est égal = " << K <<std::endl;
                                                          M=K*RT;
                                       std::cout << "  M est égal = " << M <<std::endl;

//                                    erreur_projection=(projection_en_2D - desired_position_image_).norm();
//                                              std::cout << " l'erreur_projection est égal = " << erreur_projection <<std::endl;

                                               // uh=(alpha_u_*(RT(0)*body_position_(0)+RT(1)*body_position_(1)+RT(2)*body_position_(2)+Position_(0)))/(RT(9)*body_position_(0)+RT(10)*body_position_(1)+RT(11)*body_position_(2)+Position_(2))+u0_;
}

CameraAdolcCritere::~CameraAdolcCritere ()
{
}




