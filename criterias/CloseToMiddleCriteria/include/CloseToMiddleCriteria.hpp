#ifndef  CloseToMiddleCriteria_HPP_
#define  CloseToMiddleCriteria_HPP_

#include "AbstractCriteria.h"

class CloseToMiddleCriteria: public AbstractCriteria
{   public:
	CloseToMiddleCriteria ( );
  CloseToMiddleCriteria (    double weight,
                        std::vector<MogsOptimDynamics<double> *>& dyns,
                        const QString & Robot );
    ~CloseToMiddleCriteria ();

    double compute( std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return compute<double>(dyns);
    }

    virtual void init_from_xml( QDomElement criteria,
                        std::vector<MogsOptimDynamics<double> *>& dyns );

    virtual void init_from_AbstractCriteria(  AbstractCriteria* c);

    template<typename T>
      T compute( std::vector<MogsOptimDynamics<T> *>& dyns);
      private:

        QString Robot;
		unsigned int robot_id_;
       Eigen::Matrix<double,Eigen::Dynamic,  1> qm_;
};

#include "CloseToMiddleCriteria.hxx"

#endif // CloseToMiddleCriteria_HPP_
