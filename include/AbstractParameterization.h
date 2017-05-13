#ifndef ABSTRACTPARAMETERIZATION_H
#define ABSTRACTPARAMETERIZATION_H


class AbstractParameterization
{
    public:
        AbstractParameterization();
        virtual ~AbstractParameterization();
    protected:

        unsigned int offset_;
        unsigned int nb_param_;
    private:
};

#endif // ABSTRACTPARAMETERIZATION_H
