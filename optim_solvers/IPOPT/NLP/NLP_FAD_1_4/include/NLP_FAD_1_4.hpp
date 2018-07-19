// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: NLP_FAD_1_4.hpp 1861 2010-12-21 21:34:47Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#ifndef __NLP_FAD_1_4_HPP__
#define __NLP_FAD_1_4_HPP__

#include "MogsIpoptProblem.hpp"
#include "MogsOptimDynamics.h"
#include <fadiff.h>
#include "AbstractFAD_1_4Criteria.hpp"
#include "AbstractFAD_1_4Constraint.hpp"
#include "AbstractFAD_1_4Parameterization.hpp"
#include "Dependency.h"

#ifdef MogsVisu_FOUND
#include "VisuHolder.h"
#endif // MogsVisu_FOUND

using namespace Ipopt;

class NLP_FAD_1_4:public MogsIpoptProblem
{
      public:
  /** default constructor */
	NLP_FAD_1_4 ();

  /** default destructor */
	virtual ~ NLP_FAD_1_4 ();

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


    Eigen::Matrix<double,Eigen::Dynamic,1> get_final_q(unsigned int robot_id) const
    {
        return dyns_[robot_id]->q_;
    }

    void load_xml( );

    void run_computation(const Number * x,unsigned int n, bool new_x);
    void run_gradient_computation(const Number * x, unsigned int n, bool new_x);

    virtual void set_problem_properties(const std::vector<MogsOptimDynamics<double>* >& dyns,
                                AbstractParameterization* param,
                                const std::vector<AbstractCriteria* > &criteres,
                                const std::vector<AbstractConstraint*> & constraints);

    /// load additional constraints and criterias from the xml infos
    void load_ctrs_crits(std::vector<QDomElement> & ctrs,
                         std::vector<QDomElement> & crits);

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

        std::vector<AbstractFAD_1_4Criteria* >criteres_;

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
