


template<typename T>
void ToZeroConstraint::compute( const T *x, T* g, std::vector<MogsDynamics<T> *> dyns, bool* compute_kin)
{
    int m = 1;
        g[0] = 0.;
        for (int i=0; i<n; i++)
        {
            g[0] += x[i];
        }
 //std::cout << "g[0] :" <<  g[0] << std::endl;

}
