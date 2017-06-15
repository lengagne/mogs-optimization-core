
template<typename T>
T CloseToMiddleCriteria::compute( std::vector<MogsOptimDynamics<T> *>& dyns)
{
    T Norm=0.;
    /// FIXME assume only for the first robot
	for (int i=0; i<dyns[robot_id_]->getNDof(); i++)
	{
		// norme entre qm et aq
		Norm += (qm_[i] - dyns[robot_id_]->q_[i])*(qm_[i] - dyns[robot_id_]->q_[i]);
	}
    return Norm*weight_;
}
