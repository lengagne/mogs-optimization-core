
#ifndef __MOGS_NLP_MGA_HPP__
#define __MOGS_NLP_MGA_HPP__

#include "MogsKinematics.h"
#include "MogsAbstractGeneticProblem.h"
#include "AbstractCriteria.h"
#include "AbstractConstraint.h"
#include "AbstractParameterization.h"

#ifdef MogsVisu_FOUND
#include "VisuHolder.h"
#endif

class MogsNlpMGA:  public  MogsAbstractGeneticProblem
{
      public:
//	 MogsRobotProperties robot;
  /** default constructor */
	MogsNlpMGA ();

  /** default destructor */
	virtual ~ MogsNlpMGA ();


    void get_problem_info(unsigned int & nb_variables,
                          unsigned int & nb_objectives,
                          unsigned int & nb_constraints,
                          std::vector<double>& seuils,
                          std::vector<double>& min_var,
                          std::vector<double>& max_var);

    void evaluate(  std::vector<optim_infos> &infos);

    void finalize_solution( optim_infos &info);

//    virtual void set_problem_properties(const std::vector<MogsOptimDynamics<double>* >& dyns,
//                                AbstractParameterization* param,
//                                const std::vector<AbstractCriteria* > &criteres,
//                                const std::vector<AbstractConstraint*> & constraints);

    void load_xml();

	void set_robots(const std::vector<MogsRobotProperties*> & in);

	void set_root(QDomElement root)
	{
		root_ = root;
	}


    #ifdef MogsVisu_FOUND
    void set_visu( VisuHolder * v,
                  bool during = false)
    {
        visu_optim_ = v;
        visu_during_optim_ = during;
    }
    #endif

//
//  /**@name Overloaded from TNLP */
//	//@{
//  /** Method to return some info about the nlp */
//	virtual bool get_nlp_info (Index & n, Index & m, Index & nnz_jac_g,
//				   Index & nnz_h_lag,
//				   IndexStyleEnum & index_style)=0;
//
//  /** Method to return the bounds for my problem */
//	virtual bool get_bounds_info (Index n, Number * x_l, Number * x_u,
//				      Index m, Number * g_l, Number * g_u)=0;
//
//  /** Method to return the starting point for the algorithm */
//	virtual bool get_starting_point (Index n, bool init_x, Number * x,
//					 bool init_z, Number * z_L,
//					 Number * z_U, Index m,
//					 bool init_lambda, Number * lambda)=0;
//
//  /** Method to return the objective value */
//	virtual bool eval_f (Index n, const Number * x, bool new_x,
//			     Number & obj_value)=0;
//
//  /** Method to return the gradient of the objective */
//	virtual bool eval_grad_f (Index n, const Number * x, bool new_x,
//				  Number * grad_f)=0;
//
//  /** Method to return the constraint residuals */
//	virtual bool eval_g (Index n, const Number * x, bool new_x, Index m,
//			     Number * g)=0;
//
//  /** Method to return:
//   *   1) The structure of the jacobian (if "values" is NULL)
//   *   2) The values of the jacobian (if "values" is not NULL)
//   */
//	virtual bool eval_jac_g (Index n, const Number * x, bool new_x,
//				 Index m, Index nele_jac, Index * iRow,
//				 Index * jCol, Number * values)=0;
//
//  /** @name Solution Methods */
//	//@{
//  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
//	virtual void finalize_solution (SolverReturn status,
//					Index n, const Number * x,
//					const Number * z_L,
//					const Number * z_U, Index m,
//					const Number * g,
//					const Number * lambda,
//					Number obj_value,
//					const MGAData * ip_data,
//					MGACalculatedQuantities * ip_cq) =0;
//	//@}visu_optim_
//
//    void set_robot_url(const std::vector<mogs_string> & in);
//
//     virtual  void load_xml(QDomElement criteres)=0;
//
      private:
//  /**@name Methods to block default compiler methods.
//   * The compiler automatically generates the following three methods.
//   *  Since the default compiler implementation is generally not what
//   *  you want (for all but the most simple classes), we usually
//   *  put the declarations of these methods in the private section
//   *  and never implement them. This prevents the compiler from
//   *  implementing an incorrect "default" behavior without us
//   *  knowing. (See Scott Meyers book, "Effective C++")
//   *
//   */
//	//@{
//		  MogsNlpMGA (const MogsNlpMGA &);
//          MogsNlpMGA & operator= (const MogsNlpMGA &);
//	//@}
//visu_optim_
    std::vector<MogsRobotProperties*> robots_;

    unsigned int nb_robots_;
    std::vector<MogsDynamics<double>*> dyns_;

    AbstractParameterization* parameterization_;

    std::vector<AbstractCriteria* >criteres_;

    std::vector<AbstractConstraint*> constraints_;

    QDomElement root_;

	#ifdef MogsVisu_FOUND
	VisuHolder *visu_optim_;
	bool visu_during_optim_ = false;
	#endif

};


#endif
