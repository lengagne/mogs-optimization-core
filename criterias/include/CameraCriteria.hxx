
template<typename T>
T CameraCriteria::compute( const T *x,MogsKinematics<T> *kin_)
{

     T su,sv,s,u,v;
     T obj_value=0;

     Eigen::Matrix<T, 2,1> en_2D;
     Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;

	aq_.resize(kin_->getNDof());
    for (int i=0; i<kin_->getNDof(); i++)
        aq_(i) = x[i];
    kin_->UpdateKinematicsCustom(&aq_);




    for(int i=0;i<nb_points_.size();i++)
     {
         std::cout << " body_id_["<<i<<"] = " << body_id_[i]<< std::endl;
//         std::cout << " bodyposition[i] = " << bodyposition[i]<< std::endl;

         Eigen::Matrix<T, 3, 1>Pr=kin_->getPosition(body_id_[i],bodyposition[i]);

        std::cout << "Pr(0)= " <<  Pr(0) << std::endl;
        std::cout << "Pr(1)= " <<  Pr(1) << std::endl;
        std::cout << "Pr(2)= " <<  Pr(2) << std::endl;


        Eigen::Matrix<T, 3, 3>  R = M.E.transpose().cast<T>();
        Eigen::Matrix<T, 3, 1> Image =  M.r.cast<T>() + R * Pr;


//        std::cout << "Image(0)= " <<  Image(0) << std::endl;
//        std::cout << "Image(1)= " <<  Image(1) << std::endl;
//        std::cout << "Image(2)= " <<  Image(2) << std::endl;

//         su=M(0)*Pr(0)+M(1)*Pr(1)+M(2)*Pr(2)+M(3);
//         sv=M(4)*Pr(0)+M(5)*Pr(1)+M(6)*Pr(2)+M(7);
//         s=M(8)*Pr(0)+M(9)*Pr(1)+M(10)*Pr(2)+M(11);


//         u= su/s;
//         v= sv/s;

        u = Image(0)/Image(2);
        v = Image(1)/Image(2);


                  std::cout << "u= " <<  u<< std::endl;
                  std::cout << " v = " << v<< std::endl;
                  std::cout << " desired_position_image[i](0 ) = " << desiredpositionimage[i](0)<< std::endl;
                  std::cout << " desired_position_image[i](1) = " << desiredpositionimage[i](1)<< std::endl;

           en_2D(0) = u-desiredpositionimage[i](0);
           en_2D(1) = v-desiredpositionimage[i](1);

             obj_value = obj_value + en_2D.squaredNorm() / 100;
             std::cout<<"obj_value = "<< obj_value <<std::endl<<std::endl;
    }

      return obj_value * weight_;

}

