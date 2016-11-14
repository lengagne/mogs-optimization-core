#include <CameraCriteria.hpp>


 CameraCriteria::CameraCriteria (QDomElement critere,
                                            MogsKinematics<double>* kin)

{
    QDomElement Child=critere.firstChildElement().toElement();

	QDomElement Elfocale = critere.firstChildElement("focale");
	if (!Elfocale.isNull())
	{	
		focale = Elfocale.text().trimmed().toDouble();
		qDebug()<<"focale : "<< focale;
		
	}else
	{
		std::cerr<<"Error the balise focale is not defined"<<std::endl;
		exit(0);
	}
	
	QDomElement ElKu = critere.firstChildElement("Ku");
	if (!ElKu.isNull())
	{	
		Ku = ElKu.text().trimmed().toDouble();
		qDebug()<<"Ku : "<< Ku;
		
	}else
	{
		std::cerr<<"Error the balise Ku is not defined"<<std::endl;
		exit(0);
	}	
	
	QDomElement ElKv = critere.firstChildElement("Kv");
	if (!ElKv.isNull())
	{	
		Kv = ElKv.text().trimmed().toDouble();
		qDebug()<<"Kv : "<< Kv;
		
	}else
	{
		std::cerr<<"Error the balise Kv is not defined"<<std::endl;
		exit(0);
	}
	
	QDomElement Elu0 = critere.firstChildElement("u0");
	if (!Elu0.isNull())
	{	
		u0 = Elu0.text().trimmed().toDouble();
		qDebug()<<"u0 : "<< u0;
		
	}else
	{
		std::cerr<<"Error the balise u0 is not defined"<<std::endl;
		exit(0);
	}	
	
	QDomElement Elv0 = critere.firstChildElement("v0");
	if (!Elv0.isNull())
	{	
		v0 = Elv0.text().trimmed().toDouble();
		qDebug()<<"v0 : "<< v0;
		
	}else
	{
		std::cerr<<"Error the balise v0 is not defined"<<std::endl;
		exit(0);
	}	
	
	
	QDomElement ElPosition = critere.firstChildElement("Position");
	if (!ElPosition.isNull())
	{	
		Position = ElPosition.text().trimmed();
		std::istringstream smallData (Position.toStdString(), std::ios_base::in);
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			Position_(i) = tval;
		}
		std::cout << "Position_ = " << Position_.transpose()  << std::endl;
		
	}else
	{
		std::cerr<<"Error the balise Position is not defined"<<std::endl;
		exit(0);
	}	
	
	
	QDomElement ElRotation = critere.firstChildElement("Rotation");
	if (!ElRotation.isNull())
	{	
		Rotation = ElRotation.text().trimmed();
		std::istringstream smallData (Rotation.toStdString(), std::ios_base::in);
		for (int i = 0; i < 3; i++)
		{
			smallData >> tval;
			Rotation_(i) = tval;
		}
		std::cout << "Rotation_ = " << Rotation_.transpose()  << std::endl;
		
	}else
	{
		std::cerr<<"Error the balise v0 is not defined"<<std::endl;
		exit(0);
	}		
	
	qDebug();
//          while (!Child.isNull())
//              {
// 
//                    if (Child.tagName()=="focale")
//                    {
//                         focale=Child.firstChild().toText().data().toDouble();std::cout << "   focale = " << focale << std::endl;
//                    }
// 
//                    if (Child.tagName()=="Ku")
//                    {
//                             Ku=Child.firstChild().toText().data().toDouble();std::cout << "   Ku = " << Ku  << std::endl;
//                    }
//                     if (Child.tagName()=="Kv")
//                    {
//                             Kv=Child.firstChild().toText().data().toDouble();std::cout << "   Kv = " << Kv  << std::endl;
//                    }
// 
//                     if (Child.tagName()=="u0")    {
// 
//                             u0=Child.firstChild().toText().data().toDouble();std::cout << "   u0 = " << u0  << std::endl;
// 
//                    }
//                     if (Child.tagName()=="v0")
//                    {
//                             v0=Child.firstChild().toText().data().toDouble();std::cout << "   v0 = " << v0  << std::endl;
// 
//                    }
//                    if (Child.tagName()=="Position")
//                    {
//                             Position=critere.attribute("Position");
//                             Position=Child.firstChild().toText().data();
//                             std::istringstream smallData (Position.toStdString(), std::ios_base::in);
//                                for (int i = 0; i < 3; i++)
//                                     {
//                                         smallData >> tval;
//                                         Position_(i) = tval;
//                                     }
//                             std::cout << "   Position_ = " << Position_  << std::endl;
//                    }
// 
//                    if (Child.tagName()=="Rotation")
//                    {
//                             Rotation=critere.attribute("Rotation");
//                             Rotation=Child.firstChild().toText().data();
//                             std::istringstream smallData (Rotation.toStdString(), std::ios_base::in);
//                                for (int i = 0; i < 3; i++)
//                                     {
//                                         smallData >> tval;
//                                         Rotation_(i) = tval;
//                                     }
//                             std::cout << "   Rotation_ = " << Rotation_  << std::endl;
//                    }
//                 Child = Child.nextSibling().toElement();
//              }	
	
	nb_points_ = 0;
     for (QDomElement point= critere.firstChildElement ("point"); !point.isNull();    point= point.nextSiblingElement("point"))
	{
			nb_points_++;
			mogs_string name = point.attribute("name");
			qDebug()<<"new point : "<< name;
			
			QDomElement Elrobot = point.firstChildElement("robot");
			if (!Elrobot.isNull())
			{	
				robot = Elrobot.text().trimmed();
				qDebug()<<"\trobot : "<< robot;
				
			}else
			{
				std::cerr<<"Error the balise robot is not defined (for point "<<name.toStdString()<<")"<<std::endl;
				exit(0);
			}
			
			QDomElement Elbody = point.firstChildElement("body");
			if (!Elbody.isNull())
			{	
				Body = Elbody.text().trimmed();
				qDebug()<<"\tbody : "<< Body;
				unsigned int id = kin->model->GetBodyId(Body);
				if (id ==  std::numeric_limits <unsigned int >::max () )
				{
					std::cerr << "Error the body (" <<  Body.toStdString() <<") is unkown (for point "<<name.toStdString()<<")"<< std::endl;
					exit(0);
				}
				body_id_.push_back(id);
				qDebug() <<"\tbody_id_  : " <<  id ;
			}else
			{
				std::cerr<<"Error the balise body is not defined (for point "<<name.toStdString()<<")"<<std::endl;
				exit(0);
			}
			
			QDomElement Elbody_position = point.firstChildElement("body_position");
			if (!Elbody_position.isNull())
			{	
				body_position = Elbody_position.text().trimmed();
				std::istringstream smallData (body_position.toStdString(), std::ios_base::in);

				for (int i = 0; i < 3; i++)
				{
					smallData >> tval;
					body_position_(i) = tval;
				}
				bodyposition.push_back(body_position_);
				std::cout<<"\tbody_position_ = "<< body_position_.transpose()<<std::endl;
			}else
			{
				std::cerr<<"Error the balise body_position is not defined (for point "<<name.toStdString()<<")"<<std::endl;
				exit(0);
			}	
			
			QDomElement Eldesired_position_image = point.firstChildElement("desired_position_image");
			if (!Eldesired_position_image.isNull())
			{	
				desired_position_image = Eldesired_position_image.text().trimmed();
				std::istringstream smallData (desired_position_image.toStdString(), std::ios_base::in);

				for (int i = 0; i < 2; i++)
				{
					smallData >> tval;
					desired_position_image_(i) = tval;
				}
				desiredpositionimage.push_back(desired_position_image_);
				std::cout<<"\tbody_position_ = "<< desired_position_image_.transpose()<<std::endl;
			}else
			{
				std::cerr<<"Error the balise body_position is not defined (for point "<<name.toStdString()<<")"<<std::endl;
				exit(0);
			}				

			qDebug();
	}	
		
//        while (!h.isNull())
//                             {
//                                 qDebug()<<"h.tagName() = "<< h.tagName() ;
// 
//                                  if (h.tagName()=="robot")
//                                         {
//                                      robot = h.attribute("robot");
//                                      robot= h.firstChild().toText().data();
//                                     std::cout << "   Robot  = " << robot.toStdString().c_str() << std::endl;
//                                         }
//                                  if (h.tagName()=="body")
//                                      {
//                                         Body = h.attribute("body");
//                                         Body=h.firstChild().toText().data().simplified();
// 
//                                         std::cout << "   Body  = " << Body.toStdString() << std::endl;
//                                     unsigned int id = kin->model->GetBodyId(Body);
//                                      if (id ==  std::numeric_limits <unsigned int >::max () )
//                                     {
//                                           std::cout << "   Body_ (" <<  Body.toStdString() <<") is unkown"<< std::endl;
//                                           exit(0);
//                                     }
//                                      body_id_.push_back(id);
//                                             std::cout << "   body_id_  = " <<  id << std::endl;
//                                         }
// 
// 
//                                 if (h.tagName()=="body_position")
//                                         {
//                                             body_position = h.attribute("body_position");
//                                             body_position=h.firstChild().toText().data();
//                                             std::istringstream smallData (body_position.toStdString(), std::ios_base::in);
// 
//                                                    for (int i = 0; i < 3; i++)
//                                                       {
//                                                         smallData >> tval;
//                                                         body_position_(i) = tval;
//                                                       }
//                                                 bodyposition.push_back(body_position_);
//                                                 std::cout<<"body_position_ = "<< body_position_.transpose()<<std::endl;
// 
// 
//                                      }
// 
// 
//                                         if (h.tagName()=="desired_position_image")
//                                         {
//                                             desired_position_image = h.attribute("desired_position_image");
//                                             desired_position_image=h.firstChild().toText().data();
//                                             std::istringstream smallData (desired_position_image.toStdString(), std::ios_base::in);
//                                                for (int i = 0; i < 2; i++)
//                                                       {
//                                                         smallData >> tval;
//                                                         desired_position_image_(i) = tval;
//                                                       }
//                                                     std::cout<<"desired_position_image_ = "<< desired_position_image_.transpose()<<std::endl;
//                                              desiredpositionimage.push_back(desired_position_image_);
//                                        //     std::cout << "  desired_position_image  pour le second point = " << desiredpositionimage<< std::endl;
// 
//                                         }
//                                                    h= h.nextSibling().toElement();
// 
//         }
// }
// 
//          while (!Child.isNull())
//              {
// 
//                    if (Child.tagName()=="focale")
//                    {
//                         focale=Child.firstChild().toText().data().toDouble();std::cout << "   focale = " << focale << std::endl;
//                    }
// 
//                    if (Child.tagName()=="Ku")
//                    {
//                             Ku=Child.firstChild().toText().data().toDouble();std::cout << "   Ku = " << Ku  << std::endl;
//                    }
//                     if (Child.tagName()=="Kv")
//                    {
//                             Kv=Child.firstChild().toText().data().toDouble();std::cout << "   Kv = " << Kv  << std::endl;
//                    }
// 
//                     if (Child.tagName()=="u0")    {
// 
//                             u0=Child.firstChild().toText().data().toDouble();std::cout << "   u0 = " << u0  << std::endl;
// 
//                    }
//                     if (Child.tagName()=="v0")
//                    {
//                             v0=Child.firstChild().toText().data().toDouble();std::cout << "   v0 = " << v0  << std::endl;
// 
//                    }
//                    if (Child.tagName()=="Position")
//                    {
//                             Position=critere.attribute("Position");
//                             Position=Child.firstChild().toText().data();
//                             std::istringstream smallData (Position.toStdString(), std::ios_base::in);
//                                for (int i = 0; i < 3; i++)
//                                     {
//                                         smallData >> tval;
//                                         Position_(i) = tval;
//                                     }
//                             std::cout << "   Position_ = " << Position_  << std::endl;
//                    }
// 
//                    if (Child.tagName()=="Rotation")
//                    {
//                             Rotation=critere.attribute("Rotation");
//                             Rotation=Child.firstChild().toText().data();
//                             std::istringstream smallData (Rotation.toStdString(), std::ios_base::in);
//                                for (int i = 0; i < 3; i++)
//                                     {
//                                         smallData >> tval;
//                                         Rotation_(i) = tval;
//                                     }
//                             std::cout << "   Rotation_ = " << Rotation_  << std::endl;
//                    }
//                 Child = Child.nextSibling().toElement();
//              }
// 
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
//                                        std::cout << "  M est égal = " << M <<std::endl;


//                                          for(int i=0;i<nb_points_.size();i++)
//                                    {
//                                    std::cout << " on body_id_, = " <<  body_id_[i]<< std::endl;
//                                      std::cout << " on a bodyposition  = " << bodyposition[i]<< std::endl;
//
//                                      std::cout << "on a desired_position_image  = " << desiredpositionimage[i]<< std::endl;}

	for (int i=0;i<desiredpositionimage.size();i++)
	{
		Eigen::Matrix<double,3,1> tmp;
		tmp(0) = (desiredpositionimage[i](0) - u0) * Ku;
		tmp(1) = (desiredpositionimage[i](1) - v0) * Kv;
		tmp(2) = focale;
		std::cout<<"desiredpositionimage["<<i<<"] = "<< desiredpositionimage[i].transpose()<<std::endl;
// 		std::cout<<"tmp = "<< tmp.transpose()<<std::endl;
		tmp.normalize();
		droite_point_.push_back(tmp);
	}

// 	exit(0);
									   
}

CameraCriteria::~CameraCriteria ()
{
}




