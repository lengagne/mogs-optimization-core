
template<typename T>
T PositionCriteria::compute( std::vector<MogsOptimDynamics<T> *>& dyns)
{
    T obj_value;
    Eigen::Matrix<T, 3, 1>  Pr = dyns[robot_id_]->getPosition(body_id_,body_position_);
//    std::cout<<"body_id_= "<< body_id_ <<std::endl;
//    std::cout<<"body_position_= "<< body_position_ <<std::endl;
//    for (int i=0;i<3;i++)
//    std::cout<<"Pr("<<i<<") = "<< Pr(i)<<std::endl;
    for (int i=0;i<3;i++)
		Pr(i) = Pr(i) - desired_position_(i);

    obj_value = Pr.squaredNorm();
//    std::cout<<"obj_value = " << obj_value<<std::endl<<std::endl;
    return obj_value*weight_;
}
