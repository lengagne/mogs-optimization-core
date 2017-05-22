template<typename T>
void StaticPostureParameterization::compute( const T *x,std::vector<MogsDynamics<T> *>& dyns)
{
    std::cout<<"StaticPostureParameterization compute "<<std::endl;

    T a = 3;
    a = a * x[0];
}
