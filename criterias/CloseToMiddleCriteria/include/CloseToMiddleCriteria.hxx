
template<typename T>
T CloseToMiddleCriteria::compute( const T *x,std::vector<MogsDynamics<T> *>& dyns, bool* compute_kin)
{
    T Norm=0.;
	Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;
    /// FIXME assume only for the first robot
	aq_.resize(dyns[0]->getNDof());

	for (int i=0; i<dyns[0]->getNDof(); i++)
	{
		// x va dans aq
		aq_[i] = x[i];
		// norme entre qm et aq
		Norm += (qm_[i] - aq_[i])*(qm_[i] - aq_[i]);
	}

    return Norm*weight_;

}
