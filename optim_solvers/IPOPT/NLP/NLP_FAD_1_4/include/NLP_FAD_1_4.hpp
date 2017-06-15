// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_FAD_1_4.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef __NLP_FAD_1_4_HPP__
#define __NLP_FAD_1_4_HPP__

#include "MogsNlpIpopt.hpp"
#include "MogsOptimDynamics.h"
#include <fadiff.h>
#include "AbstractFAD_1_4Critere.hpp"
#include "AbstractFAD_1_4Constraint.hpp"
#include "AbstractFAD_1_4Parameterization.hpp"
#include "Dependency.h"

using namespace Ipopt;

class NLP_FAD_1_4:public MogsNlpIpopt
{
      public:
  /** default constructor */
	NLP_FAD_1_4 ();

  /** default destructor */
	virtual ~ NLP_FAD_1_4 ();

  /**@name Overloaded from TNLP */
	//@{
  /** Method to return some info about the nlp */
	virtual bool get_nlp_info (Index & n, Index & m, Index & nnz_jac_g,
				   Index & nnz_h_lag,
				   IndexStyleEnum & index_style);

  /** Method to return the bounds for my problem */
	virtual bool get_bounds_info (Index n, Number * x_l, Number * x_u,
				      Index m, Number * g_l, Number * g_u);

  /** Method to return the starting point for the algorithm */
	virtual bool get_starting_point (Index n, bool init_x, Number * x,
					 bool init_z, Number * z_L,
					 Number * z_U, Index m,
					 bool init_lambda, Number * lambda);

  /** Method to return the objective value */
	virtual bool eval_f (Index n, const Number * x, bool new_x,
			     Number & obj_value);

  /** Method to return the gradient of the objective */
	virtual bool eval_grad_f (Index n, const Number * x, bool new_x,
				  Number * grad_f);

  /** Method to return the constraint residuals */
	virtual bool eval_g (Index n, const Number * x, bool new_x, Index m,
			     Number * g);

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
	virtual bool eval_jac_g (Index n, const Number * x, bool new_x,
				 Index m, Index nele_jac, Index * iRow,
				 Index * jCol, Number * values);

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
	virtual bool eval_h (Index n, const Number * x, bool new_x,
			     Number obj_factor, Index m,
			     const Number * lambda, bool new_lambda,
			     Index nele_hess, Index * iRow, Index * jCol,
			     Number * values);

	//@}

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
					IpoptCalculatedQuantities * ip_cq);

        void load_xml( );

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

	//  NLP_FAD_1_4();

        double tval,weight_;
        QString type,weight,name;

        NLP_FAD_1_4 (const NLP_FAD_1_4 &);
        NLP_FAD_1_4 & operator= (const NLP_FAD_1_4 &);

        std::vector<MogsOptimDynamics<Number>* > dyns_;
        std::vector<MogsOptimDynamics<F<Number> >* > adyns_;

        std::vector<Eigen::Matrix < double,Eigen::Dynamic, 1 > > q,dq,ddq;
        std::vector<Eigen::Matrix < F<double> ,Eigen::Dynamic, 1 > > aq,adq,addq;

//        std::vector<AbstractFAD_1_4Parameterization* > parameterizations_;

        AbstractFAD_1_4Parameterization* parameterization_;

        std::vector<AbstractFAD_1_4Critere* >criteres_;

        std::vector<AbstractFAD_1_4Constraint*> constraints_;

        // to remeber the dependancies
        unsigned int nnz_jac_g_;
        std::vector<unsigned int> col_,row_;

        unsigned int nb_var_;
        unsigned int nb_ctr_;       // size of      constraints_
        
        bool compute_number_, compute_gradient_;
		
		F<Number> *G, *X;
        public:

	//@}
};
#endif
