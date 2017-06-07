template<typename T>
void StaticPostureParameterization::compute( const T *x,std::vector<MogsOptimDynamics<T> *>& dyns)
{
//    std::cout<<"StaticPostureParameterization compute "<<std::endl;

    unsigned int cpt = 0;
    for (unsigned int i=0;i<nb_robots_;i++)
    {
        for (unsigned int j=0;j<ndofs_[i];j++)
        {
            dyns[i]->q_(j) = x[cpt++];
        }
    }
    for (unsigned int i=0;i<nb_robots_;i++)
        dyns[i]->UpdateStaticDynamics();
}
