#ifndef  TorqueCriteria_HPP_
#define  TorqueCriteria_HPP_

#include "AbstractCriteria.hpp"

class TorqueCriteria: public AbstractCriteria
{   public:
	TorqueCriteria ( );

    ~TorqueCriteria ();

    virtual void init_from_xml( QDomElement criteria,
                        std::vector<MogsOptimDynamics<double> *>& dyns );

    virtual void init_from_AbstractCriteria(  AbstractCriteria* c);

    double compute( std::vector<MogsOptimDynamics<double> *>& dyns)
    {
        return compute<double>(dyns);
    }

    template<typename T>
      T compute( std::vector<MogsOptimDynamics<T> *>& dyns);
      private:

	std::vector<unsigned int> robot_id_;
	unsigned int nb_robots_;
	std::vector<unsigned int> start_;
};

#include "TorqueCriteria.hxx"

#endif // TorqueCriteria_HPP_
