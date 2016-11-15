
template<typename T>
T PositionCriteria::compute( const T *x,MogsKinematics<T> *kin_, bool* compute_kin)
{
    T obj_value;

	if (*compute_kin == false)
	{
	
		Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;
		aq_.resize(kin_->getNDof());
		for (int i=0; i<kin_->getNDof(); i++)
		{
			aq_(i) = x[i];
	//		std::cout<<"x = "<< x[i]<<"\t";
		}
	//	std::cout<<std::endl;
		kin_->UpdateKinematicsCustom(&aq_);
		*compute_kin=true;
		
	}
     Eigen::Matrix<T, 3, 1>  Pr =kin_->getPosition(body_id_,body_position_);
//     std::cout<<"weight_ = "<< weight_ <<" Pr["<<body_id_<<"] = ";
    for (int i=0;i<3;i++)
	{
//		std::cout<<" "<< Pr[i]<<"\t";
		Pr(i) = Pr(i) - desired_position_(i);
	}
//	std::cout<<std::endl;
    obj_value = Pr.squaredNorm();
    return obj_value*weight_;

}