
template<typename T>
T PositionCriteria::compute( std::vector<MogsOptimDynamics<T> *>& dyns)
{
    T obj_value;
    Eigen::Matrix<T, 3, 1>  Pr =dyns[robot_id_]->getPosition(body_id_,body_position_);
    for (int i=0;i<3;i++)
		Pr(i) = Pr(i) - desired_position_(i);

    obj_value = Pr.squaredNorm();
    return obj_value*weight_;

}
