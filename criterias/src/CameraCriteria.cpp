#include <CameraCriteria.hpp>


 CameraCriteria::CameraCriteria (QDomElement critere,
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
                                qDebug()<<"h.tagName() = "<< h.tagName() ;

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
                                    unsigned int id = kin->model->GetBodyId(Body);
                                     if (id ==  std::numeric_limits <unsigned int >::max () )
                                    {
                                          std::cout << "   Body_ (" <<  Body.toStdString() <<") is unkown"<< std::endl;
                                          exit(0);
                                    }
                                     body_id_.push_back(id);
                                            std::cout << "   body_id_  = " <<  id << std::endl;
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
                                                std::cout<<"body_position_ = "<< body_position_.transpose()<<std::endl;


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
                                                    std::cout<<"desired_position_image_ = "<< desired_position_image_.transpose()<<std::endl;
                                             desiredpositionimage.push_back(desired_position_image_);
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

                   if (Child.tagName()=="Rotation")
                   {
                            Rotation=critere.attribute("Rotation");
                            Rotation=Child.firstChild().toText().data();
                            std::istringstream smallData (Rotation.toStdString(), std::ios_base::in);
                               for (int i = 0; i < 3; i++)
                                    {
                                        smallData >> tval;
                                        Rotation_(i) = tval;
                                    }
                            std::cout << "   Rotation_ = " << Rotation_  << std::endl;
                   }
                Child = Child.nextSibling().toElement();
             }

                    camera_pose_ = SpatialTransform<double>(Rotation_,Position_);
                                         std::cout << " camera_pose_= " << camera_pose_ <<std::endl;


									// focale *Ku est en général plus grand que 1
									// en général focal / taille d'un pixel sur le capteur (en m)

                                    K << focale/Ku ,  0,     u0,
                                          0,      focale/Kv ,v0,
                                          0,        0,     1;
                                       std::cout << "  K est égal = " << K<<std::endl;

                                    Eigen::Matrix<double,3,3> R = K * camera_pose_.E.transpose();
                                    Eigen::Matrix<double,3,1> P =  - K  *  camera_pose_.E* camera_pose_.r;
                                        M = SpatialTransform<double>(R,P);

//                                       M = K * camera_pose_.transpose();
//                                                          M=K*RT;
                                       std::cout << "  M est égal = " << M <<std::endl;


//                                          for(int i=0;i<nb_points_.size();i++)
//                                    {
//                                    std::cout << " on body_id_, = " <<  body_id_[i]<< std::endl;
//                                      std::cout << " on a bodyposition  = " << bodyposition[i]<< std::endl;
//
//                                      std::cout << "on a desired_position_image  = " << desiredpositionimage[i]<< std::endl;}

}

CameraCriteria::~CameraCriteria ()
{
}




