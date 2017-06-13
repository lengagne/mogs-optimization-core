
template<typename T>
void BoxContactConstraint::update_dynamics(const  T *x, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    Eigen::Matrix<T,3,1> point, point_base_coordinate, force,force_base_coordinate;
    SpatialTransform<T> trans;
    for (int i=0;i<nb_contact_;i++)
    {
        for(int j=0;j<3;j++)
        {
            point(j) = x[offset_param_ + i*6 + j];
            force(j) = x[offset_param_ + i*6 + j+3];
        }

//        for(int k=0;k<3;k++)
//            std::cout<<"point("<<k<<") = "<<point(k)<<std::endl;
//        for(int k=0;k<3;k++)
//            std::cout<<"force("<<k<<") = "<<force(k)<<std::endl;

        Eigen::Matrix<T,6,1> local_f;

        local_f.block(0,0,3,1) = point.cross(force);
        local_f.block(3,0,3,1) = force;

//        for(int k=0;k<6;k++)
//            std::cout<<"local_f("<<k<<") = "<<local_f(k)<<std::endl;

//        local_f.block(3,0,3,1) = force;

        dyns[coll_[i].robot_1]->getFrameCoordinate(0,trans);
        dyns[coll_[i].robot_1]->f_ext_[coll_[i].body_1] +=  local_f;

        dyns[coll_[i].robot_2]->getFrameCoordinate(0,trans);
        dyns[coll_[i].robot_2]->f_ext_[coll_[i].body_2] -=  local_f;
    }
}

template<typename T>
void BoxContactConstraint::compute_contact_constraint(const T*x, T *g, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    Eigen::Matrix<T,3,1> force,point, normal,contact_point1,contact_point2;
    unsigned int cpt = offset + nb_contact_;
    SpatialTransform<T> trans;
    unsigned int cpt_coll  = 0;
    for (int i=0;i<nb_body1_;i++) for (int j=0;j<nb_body2_;j++)
    {
        for(int k=0;k<3;k++)
        {
            point(k) = x[offset_param_ + 6*cpt_coll + k];
//            std::cout<<"point("<<k<<") = "<<point(k)<<std::endl;
        }


        dyns[robot1_]->getFrameCoordinate(body1_[i],trans);
//        for(int k=0;k<3;k++)
//            std::cout<<"trans.r"<<k<<" = "<<trans.r(k)<<std::endl;
        g[cpt++] = coll_detector_->compute_one_distance(trans,d1_[i],point,contact_point1);
//        std::cout<<"distance point 1 g["<<cpt-1<<"] = "<< g[cpt-1]<<std::endl;
        dyns[robot2_]->getFrameCoordinate(body2_[j],trans);
//        for(int k=0;k<3;k++)
//            std::cout<<"trans.r"<<k<<" = "<<trans.r(k)<<std::endl;
        g[cpt++] = coll_detector_->compute_one_distance(trans,d2_[j],point,contact_point2);
//        std::cout<<"distance point 2 g["<<cpt-1<<"] = "<< g[cpt-1]<<std::endl;
        for(int k=0;k<3;k++)
            force(k) = x[offset_param_ + 6*cpt_coll + 3+k];

        normal = (contact_point2 - contact_point1);
        normal.normalize();
        force.normalize();
        g[cpt++] = normal.dot(force);
//        std::cout<<"normal g["<<cpt-1<<"] = "<< g[cpt-1]<<std::endl;
        cpt_coll++;
    }
}
