#ifndef  PositionCriteria_HPP_
#define  PositionCriteria_HPP_

#include "AbstractCriteria.hpp"

class PositionCriteria: virtual public AbstractCriteria
{   public:
	PositionCriteria (QDomElement critere,
                       std::vector<MogsOptimDynamics<double> *> dyns);

    ~PositionCriteria ();

    double compute( std::vector<MogsOptimDynamics<double> *> dyns)
    {
        return compute<double>(dyns);
    }

    template<typename T>
      T compute( std::vector<MogsOptimDynamics<T> *> dyns);

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
