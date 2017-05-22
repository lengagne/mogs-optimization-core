
template<typename T>
void ToZeroConstraint::compute( T* g, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    g[0] = 0.;
    // FIXME only for first robot
    for (int i=0; i<dyns[0]->getNDof(); i++)
    {
        g[0] += dyns[0]->q_(i);
    }
}
