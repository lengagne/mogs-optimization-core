
template<typename T>
void BoxCollisionConstraint::compute( T* g, std::vector<MogsOptimDynamics<T> *>& dyns)
{
    g[offset] = 0.;
    // FIXME only for first robot
    for (int i=0; i<dyns[0]->getNDof(); i++)
    {
        g[offset] += dyns[0]->q_(i);
    }
}
