
template<typename T>
T TorqueCriteria::compute( std::vector<MogsOptimDynamics<T> *>& dyns)
{
    T Norm=0.;
    for (int i=0;i<nb_robots_;i++)
	{
		for (int j=start_[i]; j<dyns[robot_id_[i]]->getNDof(); j++)
			Norm += pow(dyns[robot_id_[i]]->q_[j],2);
	}
    return Norm*weight_;
}
