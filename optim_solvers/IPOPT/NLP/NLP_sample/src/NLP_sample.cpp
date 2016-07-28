//      Problem_Visu.cpp
//      Copyright (C) 2012 lengagne (lengagne@gmail.com) and druon
//
//      This program is free software: you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation, either version 3 of the License, or
//      (at your option) any later version.
//
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//
//      You should have received a copy of the GNU General Public License
//      along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//      This program was developped in the following labs:
//      from 2012: IUT de Beziers/ LIRMM, Beziers, France
//	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS

#include "NLP_sample.hpp"

#include <cassert>

using namespace Ipopt;

/* Constructor. */
NLP_sample::NLP_sample ()
{
}

NLP_sample::~NLP_sample ()
{
}
bool NLP_sample::get_nlp_info (Index & n, Index & m, Index & nnz_jac_g,
		     Index & nnz_h_lag, IndexStyleEnum & index_style)
{
	// The problem described in NLP_sample.hpp has 2 variables, x1, & x2,
	n = 2;

	// one equality constraint,
	m = 1;

	// 2 nonzeros in the jacobian (one for x1, and one for x2),
	nnz_jac_g = 2;

	// and 2 nonzeros in the hessian of the lagrangian
	// (one in the hessian of the objective for x2,
	//  and one in the hessian of the constraints for x1)
	nnz_h_lag = 2;

	// We use the standard fortran index style for row/col entries
	index_style = FORTRAN_STYLE;

	return true;
}

bool NLP_sample::get_bounds_info (Index n, Number * x_l, Number * x_u,
			Index m, Number * g_l, Number * g_u)
{
	// here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
	// If desired, we could assert to make sure they are what we think they are.
	assert (n == 2);
	assert (m == 1);

	// x1 has a lower bound of -1 and an upper bound of 1
	x_l[0] = -1.0;
	x_u[0] = 1.0;

	// x2 has no upper or lower bound, so we set them to
	// a large negative and a large positive number.
	// The value that is interpretted as -/+infinity can be
	// set in the options, but it defaults to -/+1e19
	x_l[1] = -1.0e19;
	x_u[1] = +1.0e19;

	// we have one equality constraint, so we set the bounds on this constraint
	// to be equal (and zero).
	g_l[0] = g_u[0] = 0.0;

	return true;
}

bool NLP_sample::get_starting_point (Index n, bool init_x, Number * x,
			   bool init_z, Number * z_L, Number * z_U,
			   Index m, bool init_lambda, Number * lambda)
{
	// Here, we assume we only have starting values for x, if you code
	// your own NLP, you can provide starting values for the others if
	// you wish.
	assert (init_x == true);
	assert (init_z == false);
	assert (init_lambda == false);

	// we initialize x in bounds, in the upper right quadrant
	x[0] = 0.5;
	x[1] = 1.5;

	return true;
}

bool NLP_sample::eval_f (Index n, const Number * x, bool new_x, Number & obj_value)
{
	// return the value of the objective function
	Number x2 = x[1];
	obj_value = -(x2 - 2.0) * (x2 - 2.0);
	return true;
}

bool NLP_sample::eval_grad_f (Index n, const Number * x, bool new_x, Number * grad_f)
{
	// return the gradient of the objective function grad_{x} f(x)

	// grad_{x1} f(x): x1 is not in the objective
	grad_f[0] = 0.0;

	// grad_{x2} f(x):
	Number x2 = x[1];
	grad_f[1] = -2.0 * (x2 - 2.0);

	return true;
}

bool NLP_sample::eval_g (Index n, const Number * x, bool new_x, Index m, Number * g)
{
	// return the value of the constraints: g(x)
	Number x1 = x[0];
	Number x2 = x[1];

	g[0] = -(x1 * x1 + x2 - 1.0);

	return true;
}

bool NLP_sample::eval_jac_g (Index n, const Number * x, bool new_x,
		   Index m, Index nele_jac, Index * iRow, Index * jCol,
		   Number * values)
{
	if (values == NULL)
	  {
		  // return the structure of the jacobian of the constraints

		  // element at 1,1: grad_{x1} g_{1}(x)
		  iRow[0] = 1;
		  jCol[0] = 1;

		  // element at 1,2: grad_{x2} g_{1}(x)
		  iRow[1] = 1;
		  jCol[1] = 2;
	  }
	else
	  {
		  // return the values of the jacobian of the constraints
		  Number x1 = x[0];

		  // element at 1,1: grad_{x1} g_{1}(x)
		  values[0] = -2.0 * x1;

		  // element at 1,2: grad_{x1} g_{1}(x)
		  values[1] = -1.0;
	  }

	return true;
}

bool NLP_sample::eval_h (Index n, const Number * x, bool new_x,
	       Number obj_factor, Index m, const Number * lambda,
	       bool new_lambda, Index nele_hess, Index * iRow,
	       Index * jCol, Number * values)
{
	if (values == NULL)
	  {
		  // return the structure. This is a symmetric matrix, fill the lower left
		  // triangle only.

		  // element at 1,1: grad^2_{x1,x1} L(x,lambda)
		  iRow[0] = 1;
		  jCol[0] = 1;

		  // element at 2,2: grad^2_{x2,x2} L(x,lambda)
		  iRow[1] = 2;
		  jCol[1] = 2;

		  // Note: off-diagonal elements are zero for this problem
	  }
	else
	  {
		  // return the values

		  // element at 1,1: grad^2_{x1,x1} L(x,lambda)
		  values[0] = -2.0 * lambda[0];

		  // element at 2,2: grad^2_{x2,x2} L(x,lambda)
		  values[1] = -2.0 * obj_factor;

		  // Note: off-diagonal elements are zero for this problem
	  }

	return true;
}

void NLP_sample::finalize_solution (SolverReturn status,
			  Index n, const Number * x, const Number * z_L,
			  const Number * z_U, Index m, const Number * g,
			  const Number * lambda, Number obj_value,
			  const IpoptData * ip_data,
			  IpoptCalculatedQuantities * ip_cq)
{
	// here is where we would store the solution to variables, or write to a file, etc
	// so we could use the solution. Since the solution is displayed to the console,
	// we currently do nothing here.
}

extern "C" NLP_sample* create()
{
    return new NLP_sample();
}

extern "C" void destroy(NLP_sample* p)
{
    delete p;
}

