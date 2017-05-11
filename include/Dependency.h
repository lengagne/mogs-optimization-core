#ifndef Dependency_H
#define Dependency_H


class Dependency
{
    public:
        Dependency(){n=0;}

        Dependency(const double &a){n=0; }

        Dependency(const Dependency &a){ n = a.n;   d = a.d;}

        virtual ~Dependency() {}

        inline bool get(unsigned int i) const
        {
//            std::cout<<"i = "<<i <<"\n n = "<<n <<std::endl;
            if(i<n) return d(i);
            return false;
        }

        /// initialize the ith value regarding a set of m values
        void init(unsigned int i, unsigned int m)
        {
            n=m;
            d.resize(m);
            for(int j=0;j<n;j++)
                d(j) = false;
            d(i) = true;
        }

        void operator = (const Dependency& a)
        {
            n = a.n;
            d = a.d;
        }

        void operator += (const Dependency& a)
        {
            if (n == 0)
            {
                n = a.n;
                d = a.d;
                return;
            }
            if(a.n == 0)
                return;
            for (int i=0;i<n;i++)
                d(i) |= a.d(i);
        }

        inline Dependency operator + (const Dependency& a) const
        {
            Dependency s=*this;
            s+= a;
            return s;
        }

        inline Dependency operator - ( ) const
        {
            return *this;
        }

        inline Dependency operator - (const Dependency& a) const
        {
            return *this+a;
        }

        inline Dependency operator * (const Dependency& a) const
        {
            return *this+a;
        }

        inline Dependency operator / (const Dependency& a) const
        {
            return *this+a;
        }

        friend std::ostream & operator<<(std::ostream &os, const Dependency& a) {
            return os << a.d.transpose();
        }

    protected:
    private:
        unsigned int n;
        Eigen::Matrix<bool, Eigen::Dynamic,1> d;
};

Dependency operator+ (const double& a, const Dependency & b)
{
    return b;
}

Dependency operator- (const double& a, const Dependency & b)
{
    return b;
}

Dependency operator* (const double& a, const Dependency & b)
{
    return b;
}

Dependency operator/ (const double& a, const Dependency & b)
{
    return b;
}

Dependency sin(const Dependency & b)
{
    return b;
}

Dependency cos(const Dependency & b)
{
    return b;
}


#endif // Dependency_H
