
template<typename T>
T CloseToMiddleCriteria::compute( std::vector<MogsOptimDynamics<T> *>& dyns)
{
    T Norm=0.;
    /// FIXME assume only for the first robot
	for (int i=0; i<dyns[0]->getNDof(); i++)
	{
		// norme entre qm et aq
		Norm += (qm_[i] - dyns[0]->q_[i])*(qm_[i] - dyns[0]->q_[i]);
	}
    return Norm*weight_;
}
