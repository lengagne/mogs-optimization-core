
template<typename T>
void ToZeroConstraint::compute( const T *x, T* g, std::vector<MogsOptimDynamics<T> *>& dyns, bool* compute_kin)
{
    g[0] = 0.;
    for (int i=0; i<n; i++)
    {
        g[0] += x[i];
    }
}
