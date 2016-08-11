#include <CameraAdolcCritere.hpp>


 CameraAdolcCritere::CameraAdolcCritere (QDomElement critere,
                                            MogsKinematics<double>* kin)

{
    QDomElement Child=critere.firstChildElement().toElement();
    QDomElement point= critere.firstChildElement ("point");
    QDomElement h=point.firstChildElement().toElement();

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

     while (!Child.isNull())
             {

                   if (Child.tagName()=="focale")
                   {
                            focale=critere.attribute("focale");
                            focale=Child.firstChild().toText().data();
                            std::istringstream smallData (focale.toStdString(), std::ios_base::in);
                                smallData >>tval;
                                focale_ = tval;
                            std::cout << "   focale_ = " << focale_  << std::endl;
                   }
                   if (Child.tagName()=="alpha_u")
                   {
                            alpha_u=critere.attribute("alpha_u");
                            alpha_u=Child.firstChild().toText().data();
                            std::istringstream smallData (alpha_u.toStdString(), std::ios_base::in);
                                        smallData >> tval;
                                        alpha_u_= tval;
                            std::cout << "   alpha_u_ = " << alpha_u_  << std::endl;

                   }

                    if (Child.tagName()=="alpha_v")
                   {
                            alpha_v=critere.attribute("alpha_v");
                            alpha_v=Child.firstChild().toText().data();
                            std::istringstream smallData (alpha_v.toStdString(), std::ios_base::in);
                                smallData >>tval;
                                alpha_v_ = tval;
                            std::cout << "   alpha_v_ = " << alpha_v_  << std::endl;
                              double alpha_v1=alpha_v_;
                   }

                    if (Child.tagName()=="u0")
                   {
                            u0=critere.attribute("u0");
                            u0=Child.firstChild().toText().data();
                            std::istringstream smallData (u0.toStdString(), std::ios_base::in);
                                smallData >>tval;
                                u0_ = tval;
                            std::cout << "   u0_ = " << u0_  << std::endl;

                   }

                    if (Child.tagName()=="v0")
                   {
                            v0=critere.attribute("v0");
                            v0=Child.firstChild().toText().data();
                            std::istringstream smallData (v0.toStdString(), std::ios_base::in);
                                smallData >>tval;
                                v0_ = tval;
                            std::cout << "   v0_ = " << v0_  << std::endl;

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
                            RotationX=critere.attribute("RotationX");
                            RotationX=Child.firstChild().toText().data();
                            std::istringstream smallData (RotationX.toStdString(), std::ios_base::in);

                                        smallData >> tval;
                                        RotationX_ = tval;
                            std::cout << "  RotationX_ = " << RotationX_  << std::endl;

                                R << 1, 0, 0,
                                     0, cos(RotationX_), -sin(RotationX_),
                                     0, sin(RotationX_), cos(RotationX_);
                                     std::cout << "  R est égal = " << R <<std::endl;

                      }
                        Child = Child.nextSibling().toElement();
                }

                                    RT << 1,          0,         0,    Position_(0),
                                          0, cos(RotationX_), -sin(RotationX_),Position_(1),
                                          0, sin(RotationX_), cos(RotationX_),Position_(2);
                                         std::cout << "  RT est égal = " << RT <<std::endl;
                                    K << alpha_u_ ,  0,     u0_,
                                          0,      alpha_v_ , v0_,
                                          0,        0,       1;
                                       std::cout << "  K est égal = " << K <<std::endl;
                                                          M=K*RT;
                                        std::cout << "  M est égal = " << M <<std::endl;
                                         su=M(0)*body_position_(0)+M(1)*body_position_(1)+M(2)*body_position_(2)+M(3);
                                         sv=M(4)*body_position_(0)+M(5)*body_position_(1)+M(6)*body_position_(2)+M(7);
                                         s=M(8)*body_position_(0)+M(9)*body_position_(1)+M(10)*body_position_(2)+M(11);
                                         u= su/s;
                                         v= sv/s;
                                         std::cout << "  u est égal = " << u  << "  v est égal = " << v  <<std::endl;
                                    projection_en_2D<<  u,
                                                        v;
                                    erreur_projection=(projection_en_2D - desired_position_image_).norm();
                                              std::cout << " l'erreur_projection est égal = " << erreur_projection <<std::endl;

                  // uh=(alpha_u_*(RT(0)*body_position_(0)+RT(1)*body_position_(1)+RT(2)*body_position_(2)+Position_(0)))/(RT(9)*body_position_(0)+RT(10)*body_position_(1)+RT(11)*body_position_(2)+Position_(2))+u0_;


}

CameraAdolcCritere::~CameraAdolcCritere ()
{
}




