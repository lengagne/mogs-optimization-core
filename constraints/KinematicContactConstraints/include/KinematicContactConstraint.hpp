#ifndef  KinematicContactConstraint_HPP_
#define  KinematicContactConstraint_HPP_

#include "AbstractConstraint.hpp"

class KinematicContactConstraint: virtual public AbstractConstraint
{   public:
	KinematicContactConstraint (QDomElement Constraint,
                          std::vector<MogsOptimDynamics<double> *>& dyns);

    ~KinematicContactConstraint ();

    void compute(double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return compute<double>(g, dyns);
    }

    template<typename T>
    void compute(T *g, std::vector<MogsOptimDynamics<T> *>& dyns);


    private:

      QString Robot1_, Robot2_, Body1_, Body2_;
      unsigned int RobotId1_, RobotId2_;
      unsigned int BodyId1_, BodyId2_;

      Eigen::Matrix<double,3,1> position_1_, position_2_;

      double radius_1_, radius_2_;

};

#include "KinematicContactConstraint.hxx"

#endif
