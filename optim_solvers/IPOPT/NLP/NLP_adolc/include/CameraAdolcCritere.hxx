
template<typename T>
T CameraAdolcCritere::compute( const T *x,MogsKinematics<T> *kin_)
{
//    T obj_value;
//
//    Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;
//    aq_.resize(kin_->getNDof());
//    for (int i=0; i<body_id_; i++)
//        aq_(i) = x[i];
//    kin_->UpdateKinematicsCustom(&aq_);
//     Eigen::Matrix<T, 3, 1>  Pr =kin_->getPosition(body_id_,body_position_);
//
//    std::cout << " getPosition= " <<Pr <<std::endl;
//    for (int i=0;i<3;i++)   Pr(i) = Pr(i) - desired_position_(i);
//    obj_value = Pr.norm();
//    return obj_value;
    return 0.0;
}
