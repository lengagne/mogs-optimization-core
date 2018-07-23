
template<typename T>
void BoxContactConstraint::update_dynamics(const  T *x, std::vector<MogsOptimDynamics<T> *>& dyns)
{

    Eigen::Matrix<T,3,1> point, point_base_coordinate, force,force_base_coordinate;
    SpatialTransform<T> trans;
    unsigned int cpt_coll  = 0;
    Eigen::Matrix<T,6,1> local_f;
    for (int i=0;i<nb_body1_;i++) for (int j=0;j<nb_body2_;j++)
    {
        for(int k=0;k<3;k++)
        {
            point(k) = x[offset_param_ + 6*cpt_coll + k];
            force(k) = x[offset_param_ + 6*cpt_coll + 3+k];
        }

        local_f.block(0,0,3,1) = point.cross(force);
        local_f.block(3,0,3,1) = force;
        if(!dyns[robot1_]->model->IsFixedBodyId(body1_[i]))
        {
//            std::cout<<"body "<< body1_[i] << " is a body id "<<std::endl;
            dyns[robot1_]->f_ext_[body1_[i]] -=  local_f;
        }
        else
        {
            /// FIXME What do if contact on fixed body ?
        }
        if(!dyns[robot2_]->model->IsFixedBodyId(body2_[j]))
        {
//            std::cout<<"body "<< body2_[j] << " is a body id "<<std::endl;
            dyns[robot2_]->f_ext_[body2_[j]] +=  local_f;
        }
        else
        {
            /// FIXME What do if contact on fixed body ?
        }
        cpt_coll++;
    }
}

template<typename T>
void BoxContactConstraint::compute_contact_constraint(const T*x, T *g, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    Eigen::Matrix<T,3,1> force,point, normal,contact_point1,contact_point2;
    unsigned int cpt = offset + nb_contact_;
    SpatialTransform<T> trans1,trans2;
    unsigned int cpt_coll  = 0;
    for (int i=0;i<nb_body1_;i++) for (int j=0;j<nb_body2_;j++)
    {
        for(int k=0;k<3;k++)
        {
            point(k) = x[offset_param_ + 6*cpt_coll + k];
            force(k) = x[offset_param_ + 6*cpt_coll + 3+k];
// 			std::cout<<"point("<<k<<") = "<< point(k)<<std::endl;
// 			std::cout<<"force("<<k<<") = "<< force(k)<<std::endl;
        }

        dyns[robot1_]->getFrameCoordinate(body1_[i],trans1);
        g[cpt++] = coll_detector_->compute_one_distance(trans1,d1_[i],point,contact_point1);
// 		std::cout<<"contrainte point 1 g["<<cpt-1<<" ] = "<<g[cpt-1]<<std::endl;
//        contact_point1 = trans1.get_Position(contact_point1);

        dyns[robot2_]->getFrameCoordinate(body2_[j],trans2);
        g[cpt++] = coll_detector_->compute_one_distance(trans2,d2_[j],point,contact_point2);
// 		std::cout<<"contrainte point 2 g["<<cpt-1<<" ] = "<<g[cpt-1]<<std::endl;
//        contact_point2 = trans2.get_Position(contact_point2);

        coll_detector_->compute_normal(trans1,trans2,d1_[i],d2_[j],normal);
// 		for(unsigned int k=0;k<3;k++)
//            std::cout<<"normal("<< k <<") = "<< normal(k)<<std::endl;
// 		for(unsigned int k=0;k<3;k++)
//            std::cout<<"force("<< k <<") = "<< force(k)<<std::endl;
// 		{
// 			std::cout<<"contact_point1("<< k <<") = "<< contact_point1(k)<<std::endl;
// 			std::cout<<"contact_point2("<< k <<") = "<< contact_point2(k)<<std::endl;
// 			std::cout<<"normal("<< k <<") = "<< normal(k)<<std::endl;
// 		}
		if(force.norm()> 0)
			force.normalize();
// 		for(unsigned int k=0;k<3;k++)
//            std::cout<<"force("<< k <<") = "<< force(k)<<std::endl;
        g[cpt++] = normal.dot(force);
// 		std::cout<<"contrainte sur l'effort g["<<cpt-1<<" ] = "<<g[cpt-1]<<std::endl;
        cpt_coll++;
    }
}
