
template<typename T>
T CameraCriteria::compute(std::vector<MogsOptimDynamics<T> *>&dyns)
{
     /// FIXME For the moment only for the first robot declared

     T su,sv,s,u,v;
     T obj_value=0;
     Eigen::Matrix<T, 2,1> en_2D;
     Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;

    for(int i=0;i<nb_points_;i++)
    {
        Eigen::Matrix<T, 3, 1>Pr=dyns[0]->getPosition(body_id_[i],bodyposition[i]);
        Eigen::Matrix<T, 3, 3>  R = camera_pose_.E.transpose().cast<T>();
		Eigen::Matrix<T, 3, 1> Image =  camera_pose_.r.cast<T>() + R * Pr;
		Eigen::Matrix<T, 3, 1> Cross;

		Cross(0) = Image(1)* droite_point_[i](2) - Image(2)* droite_point_[i](1);
		Cross(1) = Image(2)* droite_point_[i](0) - Image(0)* droite_point_[i](2);
		Cross(2) = Image(0)* droite_point_[i](1) - Image(1)* droite_point_[i](0);
		obj_value += Cross.squaredNorm(); // droite_point_[i].squaredNorm();
    }
    return obj_value * weight_;
}
