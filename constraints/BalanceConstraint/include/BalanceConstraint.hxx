
template<typename T>
void BalanceConstraint::compute( const T*x, T* g, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    unsigned int cpt = offset;
    for (int i=0;i<dyns.size();i++)
        if( dyns[i]->model->is_robot_floating_base())
        {
            for(int k=0;k<6;k++)
            {
                g[cpt++] = dyns[i]->tau_(k);
// 				std::cout<<"contrainte équilibre g["<<cpt-1<<" ] = "<<g[cpt-1]<<std::endl;
            }
        }
}
