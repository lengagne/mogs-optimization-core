
template<typename T>
void BoxContactConstraint::update_dynamics(const  T *x, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    Eigen::Matrix<T,3,1> point, force;
    for (int i=0;i<nb_contact_;i++)
    {
        for(int j=0;j<3;j++)
        {
            point(j) = x[offset + i*6 + j];
            force(j) = x[offset + i*6 + j+3];
        }
        Eigen::Matrix<T,6,1> local_f;
        local_f.block(0,0,3,1) = point.cross(force);
        local_f.block(3,0,3,1) = force;
        dyns[coll_[i].robot_1]->f_ext_[coll_[i].body_1] +=  local_f;
        dyns[coll_[i].robot_2]->f_ext_[coll_[i].body_2] -=  local_f;
    }
}

template<typename T>
void BoxContactConstraint::compute_distance_point(const T*x, T *g, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    Eigen::Matrix<T,3,1> point;
    unsigned int cpt = offset_distance_point_;
    SpatialTransform<T> trans;
    int cpt_coll  = 0;
    for (int i=0;i<nb_body1_;i++) for (int j=0;j<nb_body2_;j++)
    {
        for(int k=0;k<3;k++)
            point(k) = x[offset_param_ + 6*cpt_coll + k];

        dyns[robot1_]->getFrameCoordinate(body1_[i],trans);
        g[cpt++] = coll_detector_->compute_one_distance(trans,d1_[i],point);
        dyns[robot2_]->getFrameCoordinate(body2_[j],trans);
        g[cpt++] = coll_detector_->compute_one_distance(trans,d2_[j],point);
        cpt_coll++;
    }
}
