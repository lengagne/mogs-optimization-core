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

#include "NLP_adolc.hpp"
#include <cassert>
#include <adolc.h>
#include <adolc/adouble.h>
#include <adolc/drivers/drivers.h>
#include <adolc/taping.h>
#include <iostream>
#include <cstdlib>
#include <math.h>

using namespace Ipopt;

/* Constructor. */
NLP_adolc::NLP_adolc ()
{
    std::cout<<"Constructor of NLP_adolc"<<std::endl;
}

NLP_adolc::~NLP_adolc ()
{
}

bool NLP_adolc::get_nlp_info (Index & n, Index & m, Index & nnz_jac_g,
		     Index & nnz_h_lag, IndexStyleEnum & index_style)
{
        robot.SetRobotFile("../../mogs-all/sample-mogs/robots/KUKA_LWR/kuka_lwr.urdf");
        kin.SetRobot(&robot);
        akin.SetRobot(&robot);

        std::cout<<"Le robot a "<< robot.getNDof()<<" degres de liberté"<<std::endl;
        std::cout<<"Le robot a "<< kin.getNBodies()<<" corps"<<std::endl;
        q.resize(kin.getNDof());
        aq.resize(akin.getNDof());
        std::cout<<"akin.getNDof() =  "<<akin.getNDof()<<std::endl;
        robot.getPositionLimit(qmin,qmax);
        n=7;
        /*  Initialisation du gradient*/
        double yp = 0.0;
        adouble* x = new adouble[n];
        adouble y = 1;
        size_t tape_stats[STAT_SIZE];
                    trace_on(1);

                        for(int i=0;i<n;i++)
                        {
                            x[i] <<= i+1./2;
                            std::cout<<"on a "<<x[i] <<" v"<<std::endl;
                             y = critere(x,&akin);
                        }
                            y >>= yp;
                            delete[] x;
                    trace_off();
              tapestats(1,tape_stats);
          m = 0;
          nnz_jac_g = 0;
          nnz_h_lag = 0;
          index_style = TNLP::C_STYLE;
	return true;
}

bool NLP_adolc::get_bounds_info (Index n, Number * x_l, Number * x_u,
			Index m, Number * g_l, Number * g_u)
{

            assert(n == kin.getNDof());
            assert(m == 0);
            // the variables have lower bounds of -qmax
            for (Index i=0; i<n; i++)
            x_l[i] =qmin[i];

            // the variables have upper bounds of +qmax
            for (Index i=0; i<n; i++)
            x_u[i] = qmax[i];


	return true;
}

bool NLP_adolc::get_starting_point (Index n, bool init_x, Number * x,
			   bool init_z, Number * z_L, Number * z_U,
			   Index m, bool init_lambda, Number * lambda)
{
            assert(init_x == true);
            assert(init_z == false);
            assert(init_lambda == false);
            // initialize to the given starting point
            for(int i=0;i<7;i++)
                x[i] = i+1/2;
	return true;
}

bool NLP_adolc::eval_f (Index n, const Number * x, bool new_x, Number & obj_value)
{
            obj_value = critere(x,&kin);
	return true;
}

bool NLP_adolc::eval_grad_f (Index n, const Number * x, bool new_x, Number * grad_f)
{
	// return the gradient of the objective function grad_{x} f(x)


            assert(n == kin.getNDof());
            double* g = new double[n];
            gradient(1,n,x,g);
            for(int i(0);i<n;++i)
                grad_f[i] = g[i];

	return true;
}

bool NLP_adolc::eval_g (Index n, const Number * x, bool new_x, Index m, Number * g)
{
            assert(n == kin.getNDof());
            assert(m == 0);

	return true;
}

bool NLP_adolc::eval_jac_g (Index n, const Number * x, bool new_x,
		   Index m, Index nele_jac, Index * iRow, Index * jCol,
		   Number * values)
{
            if (values == NULL)
            {
             }
            else
            {

            }

	return true;
}

bool NLP_adolc::eval_h (Index n, const Number * x, bool new_x,
	       Number obj_factor, Index m, const Number * lambda,
	       bool new_lambda, Index nele_hess, Index * iRow,
	       Index * jCol, Number * values)
{
            if (values == NULL) {
            // return the structure. This is a symmetric matrix, fill the lower left
            // triangle only.
            // the Hessian for this problem is actually dense

            }
            else {
            // return the values. This is a symmetric matrix, fill the lower left
            // triangle only
            // fill the objective portion

            }

	return true;
}

void NLP_adolc::finalize_solution (SolverReturn status,
			  Index n, const Number * x, const Number * z_L,
			  const Number * z_U, Index m, const Number * g,
			  const Number * lambda, Number obj_value,
			  const IpoptData * ip_data,
			  IpoptCalculatedQuantities * ip_cq)
{
            // For this example, we write the solution to the console
            printf("\n\nSolution of the primal variables, x\n");
            for (Index i=0; i<n; i++) {
            printf("x[%d] = %e\n", i, x[i]);
            }
            printf("\n\nSolution of the bound multipliers, z_L and z_U\n");
            for (Index i=0; i<n; i++) {
            printf("z_L[%d] = %e\n", i, z_L[i]);
            }
            for (Index i=0; i<n; i++) {
            printf("z_U[%d] = %e\n", i, z_U[i]);
            }
            printf("\n\nObjective value\n");
            printf("f(x*) = %e\n", obj_value);
}

// For dynamic loading of the library : do not remove !!

extern "C" NLP_adolc* create()
{
    return new NLP_adolc();
}

extern "C" void destroy(NLP_adolc* p)
{
    delete p;
}


