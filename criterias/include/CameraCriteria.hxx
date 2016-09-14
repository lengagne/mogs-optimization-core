
template<typename T>
T CameraCriteria::compute( const T *x,MogsKinematics<T> *kin_, bool* compute_kin)
{

     T su,sv,s,u,v;
     T obj_value=0;

     Eigen::Matrix<T, 2,1> en_2D;
     Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;

// 	 if (*compute_kin == false)
	 {
		aq_.resize(kin_->getNDof());
// 		std::cout<<"q =";
		for (int i=0; i<kin_->getNDof(); i++)
		{
// 			std::cout<<x[i]<<" ";
			aq_(i) = x[i];
		}
// 		std::cout<<std::endl;
		kin_->UpdateKinematicsCustom(&aq_);
		*compute_kin = true;
		
	 }


    for(int i=0;i<nb_points_;i++)
     {
//          std::cout << " body_id_["<<i<<"] = " << body_id_[i]<< std::endl;
//         std::cout << " bodyposition[i] = " << bodyposition[i].transpose()<< std::endl;

         Eigen::Matrix<T, 3, 1>Pr=kin_->getPosition(body_id_[i],bodyposition[i]);
// 			std::cout<<"Pr = "<< Pr(0)<<" "<< Pr(1)<<" "<<Pr(2)<<std::endl;

//         Eigen::Matrix<T, 3, 3>  R = M.E.transpose().cast<T>();
//         Eigen::Matrix<T, 3, 1> Image =  M.r.cast<T>() + R * Pr;

//         su=M(0)*Pr(0)+M(1)*Pr(1)+M(2)*Pr(2)+M(3);
//         sv=M(4)*Pr(0)+M(5)*Pr(1)+M(6)*Pr(2)+M(7);
//         s=M(8)*Pr(0)+M(9)*Pr(1)+M(10)*Pr(2)+M(11);
//         u= su/s;
//         v= sv/s;

		
// 		u = Image(0)/Image(2);
// 		v = Image(1)/Image(2);
// 		en_2D(0) = u-desiredpositionimage[i](0);
// 		en_2D(1) = v-desiredpositionimage[i](1);


// 		en_2D(0) = Image(0)-desiredpositionimage[i](0)*Image(2);
// 		en_2D(1) = Image(1)-desiredpositionimage[i](1)*Image(2);
// 		
// 		
//      obj_value = obj_value + en_2D.squaredNorm();

        Eigen::Matrix<T, 3, 3>  R = camera_pose_.E.transpose().cast<T>();
		Eigen::Matrix<T, 3, 1> Image =  camera_pose_.r.cast<T>() + R * Pr;
// 		std::cout<<"Image = "<< Image(0)<<" "<< Image(1)<<" "<<Image(2)<<std::endl;
		Eigen::Matrix<T, 3, 1> Cross;
		
		Cross(0) = Image(1)* droite_point_[i](2) - Image(2)* droite_point_[i](1);
		Cross(1) = Image(2)* droite_point_[i](0) - Image(0)* droite_point_[i](2);
		Cross(2) = Image(0)* droite_point_[i](1) - Image(1)* droite_point_[i](0);
// 		obj_value += Cross.squaredNorm(); // droite_point_[i].squaredNorm();
		obj_value += Cross.norm(); // droite_point_[i].squaredNorm();
		
// 		std::cout<<"tmp("<<i<<") = "<< Cross.squaredNorm()<< " obj_value = "<< obj_value <<std::endl <<std::endl;
		
    }

      return obj_value * weight_;

}

