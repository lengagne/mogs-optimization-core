// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_sample.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef __NLP_sample_HPP__
#define __NLP_sample_HPP__

#include "MogsNlpIpopt.hpp"

using namespace Ipopt;

/** C++ Example NLP for interfacing a problem with IPOPT.
 *  NLP_sample implements a C++ example showing how to interface with IPOPT
 *  through the TNLP interface. This example is designed to go along with
 *  the tutorial document (see Examples/CppTutorial/).
 *  This class implements the following NLP.
 *
 * min_x f(x) = -(x2-2)^2
 *  s.t.
 *       0 = x1^2 + x2 - 1
 *       -1 <= x1 <= 1
 *
 */
class NLP_sample:public MogsNlpIpopt
{
      public:
  /** default constructor */
	NLP_sample ();

  /** default destructor */
	virtual ~ NLP_sample ();

   void load_xml( );

  /**@name Overloaded from TNLP */
	//@{
  /** Method to return some info about the nlp */
	virtual bool get_nlp_info (Ipopt::Index & n, Ipopt::Index & m, Ipopt::Index & nnz_jac_g,
				   Ipopt::Index & nnz_h_lag,
				   IndexStyleEnum & index_style);

  /** Method to return the bounds for my problem */
	virtual bool get_bounds_info (Ipopt::Index n, Number * x_l, Number * x_u,
				      Ipopt::Index m, Number * g_l, Number * g_u);

  /** Method to return the starting point for the algorithm */
	virtual bool get_starting_point (Ipopt::Index n, bool init_x, Number * x,
					 bool init_z, Number * z_L,
					 Number * z_U, Ipopt::Index m,
					 bool init_lambda, Number * lambda);

  /** Method to return the objective value */
	virtual bool eval_f (Ipopt::Index n, const Number * x, bool new_x,
			     Number & obj_value);

  /** Method to return the gradient of the objective */
	virtual bool eval_grad_f (Ipopt::Index n, const Number * x, bool new_x,
				  Number * grad_f);

  /** Method to return the constraint residuals */
	virtual bool eval_g (Ipopt::Index n, const Number * x, bool new_x, Ipopt::Index m,
			     Number * g);

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
	virtual bool eval_jac_g (Ipopt::Index n, const Number * x, bool new_x,
				 Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index * iRow,
				 Ipopt::Index * jCol, Number * values);

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
	virtual bool eval_h (Ipopt::Index n, const Number * x, bool new_x,
			     Number obj_factor, Ipopt::Index m,
			     const Number * lambda, bool new_lambda,
			     Ipopt::Index nele_hess, Ipopt::Index * iRow, Ipopt::Index * jCol,
			     Number * values);

	//@}

  /** @name Solution Methods */
	//@{
  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
	virtual void finalize_solution (SolverReturn status,
					Ipopt::Index n, const Number * x,
					const Number * z_L,
					const Number * z_U, Ipopt::Index m,
					const Number * g,
					const Number * lambda,
					Number obj_value,
					const IpoptData * ip_data,
					IpoptCalculatedQuantities * ip_cq);
	//@}

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
	//  NLP_sample();
	  NLP_sample (const NLP_sample &);
	  NLP_sample & operator= (const NLP_sample &);
	//@}
};


#endif
