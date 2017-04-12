
template<typename T>
T CloseToMiddleCriteria::compute( const T *x,MogsKinematics<T> *kin_, bool* compute_kin)
{
    T Norm=0.;
	Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;
	aq_.resize(kin_->getNDof());

	for (int i=0; i<kin_->getNDof(); i++)
	{
		// x va dans aq
		aq_[i] = x[i];
		// norme entre qm et aq
		Norm += (qm_[i] - aq_[i])*(qm_[i] - aq_[i]);

	}

    return Norm*weight_;

}
