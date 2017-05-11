


template<typename T>
void PositionDConstraint::compute( const T *x, T* g, MogsKinematics<T> *kin_, bool* compute_kin)
{
//T obj_value;
	if (*compute_kin == false)
	{
		Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;
		aq_.resize(kin_->getNDof());
		for (int i=0; i<kin_->getNDof(); i++)
		{
			aq_(i) = x[i];
		}
		kin_->UpdateKinematicsCustom(&aq_);
		*compute_kin = true;
	}

    Eigen::Matrix<T, 3, 1>  Pr =kin_->getPosition(body_id_,body_Position_);
    for (int i=0;i<3;i++)
	{
		g[i] = Pr(i);
//        std::cout << "PositionDConstraint::compute g[" << i << "] :" <<  g[i] << std::endl;
	}
}
