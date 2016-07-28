
template<typename T>
T PositionAdolcCritere::compute( const T *x,MogsKinematics<T> *kin_)
{
    T obj_value;

     Eigen::Matrix<T, 3, 1>  Pd(desired_position_(0),desired_position_(1),desired_position_(2));

    Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;
    aq_.resize(kin_->getNDof());
    for (int i=0; i<7; i++)
        aq_(i) = x[i];
    kin_->UpdateKinematicsCustom(&aq_);
     Eigen::Matrix<T, 3, 1>  Pr =kin_->getPosition(body_id_,  Eigen::Matrix<double, 3, 1>::Zero());
    for (int i=0;i<7;i++)
    obj_value = (Pr - Pd).norm();
    return obj_value;

}
