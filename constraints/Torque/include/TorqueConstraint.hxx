
template<typename T>
void TorqueConstraint::compute(T* g, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    unsigned int cpt = offset;
    for (int i=0;i<robot_id_.size();i++)
    {
        for (int j=start_[i];j<end_[i];j++)
            g[cpt++] = dyns[i]->tau_(j);
    }
}
