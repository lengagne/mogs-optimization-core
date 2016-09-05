#include <CameraAdolcCritere.hpp>


 CameraAdolcCritere::CameraAdolcCritere (QDomElement critere,
                                            MogsKinematics<double>* kin)

{
    QDomElement Child=critere.firstChildElement().toElement();



     for (QDomElement point= critere.firstChildElement ("point"); !point.isNull();    point= point.nextSiblingElement("point"))
	{
                                      QDomElement h=point.firstChildElement().toElement();
                                     if (point.tagName()=="point")
                                    nb_points_.push_back(nb_points);
                                    nb_points++;
                                    std::cout << "le vecteur contient " << nb_points_.size() << " points.\n";

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

                                      body_id_.push_back(kin->model->GetBodyId(Body));

                                        if(nb_points==1)
                                                      {
                                    if (body_id_[0] ==  std::numeric_limits <unsigned int >::max () )
                                            {
                                          std::cout << "   Body_ (" <<  Body.toStdString() <<") is unkown"<< std::endl;
                                          exit(0);
                                            }
                                                std::cout << "   body_id_  = " <<  body_id_[0] << std::endl;
                                            }

                                     if(nb_points==2)
                                            {
                                    if (body_id_[1] ==  std::numeric_limits <unsigned int >::max () )
                                            {
                                          std::cout << "   Body_ (" <<  Body.toStdString() <<") is unkown"<< std::endl;
                                          exit(0);
                                            }
                                                std::cout << "   body_id_  = " <<  body_id_[1] << std::endl;
                                            }
                                            if(nb_points==3)
                                            {
                                    if (body_id_[2] ==  std::numeric_limits <unsigned int >::max () )
                                            {
                                          std::cout << "   Body_ (" <<  Body.toStdString() <<") is unkown"<< std::endl;
                                          exit(0);
                                            }
                                                std::cout << "   body_id_  = " <<  body_id_[2] << std::endl;
                                            }

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
                                                bodyposition.push_back(body_position_);

                                             if(nb_points==1)
                                                      {
                                          std::cout << "   body_position pour le premier point est = " << bodyposition[0] << std::endl;

                                          }
                                             if(nb_points==2)
                                                      {
                                          std::cout << "   body_position pour le deuxième point est = " << bodyposition[1] << std::endl;}
                                            if(nb_points==3)
                                                      {
                                          std::cout << "   body_position pour le troisième point est = " << bodyposition[2] << std::endl;}

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
                                             desiredpositionimage.push_back(desired_position_image_);
                                         if(nb_points==1)
                                                      {
                                          std::cout << "   desired_position_image  pour le premier point est = " << desiredpositionimage[0] << std::endl;

                                          }
                                          if(nb_points==2)
                                                      {
                                          std::cout << "   desired_position_image  pour le deuxième point est = " << desiredpositionimage[1] << std::endl;

                                          }
                                           if(nb_points==3)
                                                      {
                                         std::cout << "   desired_position_image  pour le troisième point est = " << desiredpositionimage[2] << std::endl;}


                                       //     std::cout << "  desired_position_image  pour le second point = " << desiredpositionimage<< std::endl;

                                        }
                                                   h= h.nextSibling().toElement();

        }
}

         while (!Child.isNull())
             {

                   if (Child.tagName()=="focale")
                   {
                        focale=Child.firstChild().toText().data().toDouble();std::cout << "   focale = " << focale << std::endl;
                   }

                   if (Child.tagName()=="Ku")
                   {
                            Ku=Child.firstChild().toText().data().toDouble();std::cout << "   Ku = " << Ku  << std::endl;
                   }
                    if (Child.tagName()=="Kv")
                   {
                            Kv=Child.firstChild().toText().data().toDouble();std::cout << "   Kv = " << Kv  << std::endl;
                   }

                    if (Child.tagName()=="u0")    {

                            u0=Child.firstChild().toText().data().toDouble();std::cout << "   u0 = " << u0  << std::endl;

                   }
                    if (Child.tagName()=="v0")
                   {
                            v0=Child.firstChild().toText().data().toDouble();std::cout << "   v0 = " << v0  << std::endl;

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
                            //RotationX=critere.attribute("RotationX");
                            RotationX=Child.firstChild().toText().data().toDouble();
                            std::cout << "  RotationX = " << RotationX  << std::endl;

                                RX << 1, 0, 0,
                                     0, cos(RotationX), -sin(RotationX),
                                     0, sin(RotationX), cos(RotationX);
                                     std::cout << "  Rotation sur X est égal = " << RX <<std::endl;

                      }
                       if (Child.tagName()=="RotationY")
                   {

                            RotationY=Child.firstChild().toText().data().toDouble();
                            std::cout << "  RotationY = " << RotationY  << std::endl;

                                RY << cos(RotationY),    0, sin(RotationY),
                                            0,          1,         0,
                                     -sin(RotationY),   0, cos(RotationY);
                                     std::cout << "  Rotation sur Y est égal = " << RY <<std::endl;

                      }
                         if (Child.tagName()=="RotationZ")
                   {

                            RotationZ=Child.firstChild().toText().data().toDouble();
                            std::cout << "  RotationZ = " << RotationZ  << std::endl;

                                RZ << cos(RotationZ), -sin(RotationZ), 0,
                                      sin(RotationZ),  cos(RotationZ), 0,
                                        0,           0,               1;
                             std::cout << "  Rotation sur Z est égal = " << RZ <<std::endl;

                      }
                        Child = Child.nextSibling().toElement();
                }


                                    RT << 1,          0,         0,          Position_(0),
                                          0, cos(RotationX), -sin(RotationX),Position_(1),
                                          0, sin(RotationX), cos(RotationX), Position_(2);
                                         std::cout << "  RT est égal = " << RT <<std::endl;


                                    K << focale*Ku ,  0,     u0,
                                          0,      focale*Kv ,v0,
                                          0,        0,     1;
                                       std::cout << "  K est égal = " << K<<std::endl;
                                                          M=K*RT;
                                       std::cout << "  M est égal = " << M <<std::endl;


//                                          for(int i=0;i<nb_points_.size();i++)
//                                    {
//                                    std::cout << " on body_id_, = " <<  body_id_[i]<< std::endl;
//                                      std::cout << " on a bodyposition  = " << bodyposition[i]<< std::endl;
//
//                                      std::cout << "on a desired_position_image  = " << desiredpositionimage[i]<< std::endl;}

}

CameraAdolcCritere::~CameraAdolcCritere ()
{
}




