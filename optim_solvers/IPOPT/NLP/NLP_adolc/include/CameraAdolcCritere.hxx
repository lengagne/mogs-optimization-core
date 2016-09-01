
template<typename T>
T CameraAdolcCritere::compute( const T *x,MogsKinematics<T> *kin_)
{

     T su,sv,s,u,v,l;
     T obj_value=0;

     Eigen::Matrix<T, 2,1> en_2D;
     Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;

	aq_.resize(kin_->getNDof());
    for (int i=0; i<kin_->getNDof(); i++)
        aq_(i) = x[i];
    kin_->UpdateKinematicsCustom(&aq_);

//    for(int i=0;i<nb_points_;i++)
// {
// 	
// 	std::cout<<"body_id_ = "<< body_id_ <<std::endl;
// 	std::cout<<"body_position_ = "<< body_position_ <<std::endl;
     Eigen::Matrix<T, 3, 1>Pr=kin_->getPosition(body_id_,body_position_);


     su=M(0)*Pr(0)+M(1)*Pr(1)+M(2)*Pr(2)+M(3);
	 sv=M(4)*Pr(0)+M(5)*Pr(1)+M(6)*Pr(2)+M(7);
     s=M(8)*Pr(0)+M(9)*Pr(1)+M(10)*Pr(2)+M(11);
     u= su/s;
     v= sv/s;

<<<<<<< HEAD
//        std::cout << "  su = " << su << std::endl;
//        std::cout << "  sv = " << sv << std::endl;
//        std::cout << "  s = " << s << std::endl;
//        std::cout << "  u = " << u << std::endl;
//        std::cout << "  v = " << v << std::endl;

       en_2D(0) = u-desired_position_image_(0);
       en_2D(1) = v-desired_position_image_(1);

        obj_value = obj_value + en_2D.norm();
//}
      return obj_value;
=======
       en_2D(0) = u-desired_position_image_(0);
       en_2D(1) = v-desired_position_image_(1);
        obj_value = obj_value + en_2D.norm();
      return obj_value * weight_;
>>>>>>> 7217eba95b3146b1b35b18fa50208f9b6c9c7c39
}

