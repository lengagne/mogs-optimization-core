
template<typename T>
void BalanceConstraint::compute( const T*x, T* g, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    unsigned int cpt = offset;
    for (int i=0;i<dyns.size();i++)
        if( dyns[i]->model->is_robot_floating_base())
        {
            for(int k=0;k<6;k++)
            {
				// wr work on internal f, because if we work on torque the optim might go to singularities
                g[cpt++] = dyns[i]->model->f[1](k);
// 				std::cout<<"contrainte équilibre g["<<cpt-1<<" ] = "<<g[cpt-1]<<std::endl;
            }
        }
}
