
template<typename T>
T PositionAdolcCritere::compute( const T *x,MogsKinematics<T> *kin_)
{
    T obj_value;
    typedef Eigen::Matrix<T, 3, 1> Vec;
    Vec Pd = Vec(0.5, 0.5, 0.5);
    Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;
    aq_.resize(kin_->getNDof());
    for (int i=0; i<7; i++)
        aq_(i) = x[i];
    kin_->UpdateKinematicsCustom(&aq_);
    Vec Pr =kin_->getPosition(7,  Eigen::Matrix<double, 3, 1>::Zero());
    for (int i=0;i<7;i++)
    obj_value = (Pr - Pd).norm();
    return obj_value;

}
