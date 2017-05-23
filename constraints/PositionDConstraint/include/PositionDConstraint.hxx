
template<typename T>
void PositionDConstraint::compute(T* g, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    Eigen::Matrix<T, 3, 1>  Pr =dyns[robot_id_]->getPosition(body_id_,body_Position_);
    for (int i=0;i<3;i++)
	{
		g[offset+i] = Pr(i);
//        std::cout << "PositionDConstraint::compute g[" << i << "] :" <<  g[i] << std::endl;
	}
}
