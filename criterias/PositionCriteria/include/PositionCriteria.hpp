#ifndef  PositionCriteria_HPP_
#define  PositionCriteria_HPP_

#include "AbstractCriteria.hpp"

class PositionCriteria: virtual public AbstractCriteria
{   public:
	PositionCriteria (QDomElement critere,
                       std::vector<MogsOptimDynamics<double> *> dyns);

    ~PositionCriteria ();

    double compute( const double *x , std::vector<MogsOptimDynamics<double> *> dyns, bool* compute_kin)
    {
        return compute<double>(x,dyns, compute_kin);
    }

    template<typename T>
      T compute( const T *x, std::vector<MogsOptimDynamics<T> *> dyns, bool* compute_kin);

      private:

        QString Body;
        QString Robot;
        QString desired_position;
        QString body_position;
        double tval;

        unsigned int robot_id_;
        unsigned int body_id_;

        Eigen::Matrix<double, 3, 1> body_position_;
        Eigen::Matrix<double, 3, 1> desired_position_;

};

#include "PositionCriteria.hxx"

#endif // PositionCriteria_HPP_
