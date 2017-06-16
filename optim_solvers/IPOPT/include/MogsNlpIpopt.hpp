// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: MogsNlpIpopt.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef __MOGS_NLP_IPOPT_HPP__
#define __MOGS_NLP_IPOPT_HPP__
#include "MogsOptimDynamics.h"
#include "MogsAbstractOptimization.h"
#include "IpTNLP.hpp"
using namespace Ipopt;

class MogsNlpIpopt:public TNLP
{
      public:
  /** default constructor */
	MogsNlpIpopt ();

  /** default destructor */
	virtual ~ MogsNlpIpopt ();

  /**@name Overloaded from TNLP */
	//@{
  /** Method to return some info about the nlp */
	virtual bool get_nlp_info (Index & n, Index & m, Index & nnz_jac_g,
				   Index & nnz_h_lag,
				   IndexStyleEnum & index_style)=0;

  /** Method to return the bounds for my problem */
	virtual bool get_bounds_info (Index n, Number * x_l, Number * x_u,
				      Index m, Number * g_l, Number * g_u)=0;

  /** Method to return the starting point for the algorithm */
	virtual bool get_starting_point (Index n, bool init_x, Number * x,
					 bool init_z, Number * z_L,
					 Number * z_U, Index m,
					 bool init_lambda, Number * lambda)=0;

  /** Method to return the objective value */
	virtual bool eval_f (Index n, const Number * x, bool new_x,
			     Number & obj_value)=0;

  /** Method to return the gradient of the objective */
	virtual bool eval_grad_f (Index n, const Number * x, bool new_x,
				  Number * grad_f)=0;

  /** Method to return the constraint residuals */
	virtual bool eval_g (Index n, const Number * x, bool new_x, Index m,
			     Number * g)=0;

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
	virtual bool eval_jac_g (Index n, const Number * x, bool new_x,
				 Index m, Index nele_jac, Index * iRow,
				 Index * jCol, Number * values)=0;

  /** @name Solution Methods */
	//@{
  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
	virtual void finalize_solution (SolverReturn status,
					Index n, const Number * x,
					const Number * z_L,
					const Number * z_U, Index m,
					const Number * g,
					const Number * lambda,
					Number obj_value,
					const IpoptData * ip_data,
					IpoptCalculatedQuantities * ip_cq) =0;
	//@}

	void save_results( 	Index n,
						const Number* x,
						Number obj_value);

	void set_root(QDomElement root)
	{
		root_ = root;
	}

    virtual void set_problem_properties(const std::vector<MogsOptimDynamics<double>* >& dyns,
                                        const AbstractParameterization* param,
                                        const std::vector<AbstractCriteria* > &criteres,
                                        const std::vector<AbstractConstraint*> & constraints)
    {

    }

    void set_robots(const std::vector<MogsRobotProperties*> & in);

     virtual  void load_xml( )=0;

      private:
  /**@name Methods to block default compiler methods.
   * The compiler automatically generates the following three methods.
   *  Since the default compiler implementation is generally not what
   *  you want (for all but the most simple classes), we usually
   *  put the declarations of these methods in the private section
   *  and never implement them. This prevents the compiler from
   *  implementing an incorrect "default" behavior without us
   *  knowing. (See Scott Meyers book, "Effective C++")
   *
   */
	//@{
		  MogsNlpIpopt (const MogsNlpIpopt &);
          MogsNlpIpopt & operator= (const MogsNlpIpopt &);
	//@}
    protected:
     unsigned int nb_robots_;
	 std::vector<MogsRobotProperties*> robots_;

	 QDomElement root_;

};


#endif
