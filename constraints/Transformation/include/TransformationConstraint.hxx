
template<typename T>
void TransformationConstraint::compute(T* g, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    SpatialTransform<T> t1,t2,t;
    dyns[robot1_id_]->getFrameCoordinate(body1_id_,t1);
    dyns[robot2_id_]->getFrameCoordinate(body2_id_,t2);

    t = t2.transpose() * Transformation_ * t1;
    Eigen::Matrix < T, 3, 1 > r,p;
    t.get_RPY(r,p);
    for (int i=0;i<3;i++)
    {
        g[offset+i] = r(i);
        g[offset+i+3] = p(i);
    }
}
