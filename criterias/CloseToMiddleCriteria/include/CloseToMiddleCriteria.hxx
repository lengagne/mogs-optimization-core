
template<typename T>
T CloseToMiddleCriteria::compute( const T *x,MogsKinematics<T> *kin_, bool* compute_kin)
{
    //T obj_value;
    T Norm=0.;
    //double oldNorm;
    //double derive;
//	if (*compute_kin == false)
	//{

		Eigen::Matrix < T,Eigen::Dynamic, 1 > aq_;
		aq_.resize(kin_->getNDof());

		for (int i=0; i<kin_->getNDof(); i++)
		{
            // x va dans aq
            aq_[i] = x[i];
            // norme entre qm et aq
            Norm += (qm[i] - aq_[i])*(qm[i] - aq_[i]);


            //oldNorm = Norm;
           // derive = Norm - oldNorm

        }

		/*


		std::vector<double> qmin_;
		std::vector<double> qmax_;
		prop->getPositionLimit(qmin_,qmax_);
		for (int i=0; i<kin_->getNDof(); i++)
		{
			aq_(i) = x[i];
	//		std::cout<<"x = "<< x[i]<<"\t";
		}
	//	std::cout<<std::endl;
		kin_->UpdateKinematicsCustom(&aq_);
		*compute_kin=true;
*/
	//}
    /* Eigen::Matrix<T, 3, 1>  Pr =kin_->getclose_to_middle(body_id_,body_close_to_middle_);
//     std::cout<<"weight_ = "<< weight_ <<" Pr["<<body_id_<<"] = ";
    for (int i=0;i<3;i++)
	{
//		std::cout<<" "<< Pr[i]<<"\t";
		Pr(i) = Pr(i) - desired_close_to_middle_(i);
	}
//	std::cout<<std::endl;
    obj_value = Pr.squaredNorm();
  */
    //return obj_value*weight_;
    return Norm;

}
