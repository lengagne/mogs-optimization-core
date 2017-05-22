
template<typename T>
void PositionDConstraint::compute( const T *x, T* g, std::vector<MogsOptimDynamics<T> *>& dyns, bool* compute_kin)
{
//T obj_value;
	/// Assume done before
	/*
	if (*compute_kin == false)
	{*/
//		Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;
//		aq_.resize(dyns[robot_id_]->getNDof());
//		for (int i=0; i<dyns[robot_id_]->getNDof(); i++)
//		{
//			aq_(i) = x[i];
//		}
//		dyns[robot_id_]->UpdateKinematicsCustom(&aq_);
//		*compute_kin = true;
	/*}*/

    Eigen::Matrix<T, 3, 1>  Pr =dyns[robot_id_]->getPosition(body_id_,body_Position_);
    for (int i=0;i<3;i++)
	{
		g[i] = Pr(i);
//        std::cout << "PositionDConstraint::compute g[" << i << "] :" <<  g[i] << std::endl;
	}
}
