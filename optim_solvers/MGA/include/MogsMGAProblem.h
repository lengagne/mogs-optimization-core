#ifndef __MOGS_MGA_PROBLEM_HPP__
#define __MOGS_MGA_PROBLEM_HPP__

#include "MogsAbstractGeneticProblem.h"
#include "AbstractOptimizationProblem.h"

#ifdef MogsVisu_FOUND
#include "VisuHolder.h"
#endif

class MogsMGAProblem:  public  MogsAbstractGeneticProblem,  public AbstractOptimizationProblem
{
      public:
//	 MogsRobotProperties robot;
  /** default constructor */
	MogsMGAProblem ();

  /** default destructor */
	virtual ~ MogsMGAProblem ();

    void get_problem_info(unsigned int & nb_variables,
                          unsigned int & nb_objectives,
                          unsigned int & nb_constraints,
                          std::vector<double>& seuils,
                          std::vector<double>& min_var,
                          std::vector<double>& max_var,
                          std::vector<double>& min_ctr,
                          std::vector<double>& max_ctr);

    void evaluate(  std::vector<optim_infos> &infos);

    void finalize_solution( optim_infos &info);

    virtual  void load_xml( );

      private:

    std::vector<MogsOptimDynamics<double>*> dyns_;

    AbstractParameterization* parameterization_;

    std::vector<AbstractCriteria* >criteres_;

    std::vector<AbstractConstraint*> constraints_;

};


#endif
