
template<typename T>
T ToZeroConstraint::compute( const T *x,MogsKinematics<T> *kin_, bool* compute_kin)
{

    g[0] = 0.;
    for (int i=0; i<n; i++)
    {
        g[0] += x[i];
    }

    for(j = 0; j < m; j++)
        for (int i=0; i<n; i++)
        {
            iRow[i] = j;
            jCol[i] = i;
        }


    for (int i=0; i<n; i++)
    {
        values[i] = 1.0;
    }

    return g[0] ;

}
