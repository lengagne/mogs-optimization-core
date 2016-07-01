//      MogsIpoptOptimization.h
//      Copyright (C) 2012 lengagne (lengagne@gmail.com)
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
//      2009-2011:  Joint robotics Laboratory - CNRS/AIST,Tsukuba, Japan.
//      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
//      2012-2013:  IUT de Beziers/ LIRMM, Beziers, France
//	from 2013:  Université Blaise Pascal / axis : ISPR / theme MACCS

#ifndef __MogsIpoptOptimization__
#define __MogsIpoptOptimization__

#include "MogsAbstractOptimization.h"

#include "config_MogsIpoptOptimizationCore.h"

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"


#include "IpTNLP.hpp"

using namespace Ipopt;

class MogsIpoptOptimization: public MogsAbstractOptimization
{
      public:

	MogsIpoptOptimization();

	~MogsIpoptOptimization();


	/** This function get the current time,
	 *	the current values of joint position, velocity, acceleration and torques
	 * 	and compute the needed values
	 * 	The function returns false when the pattern is ended.
	 */

        void read_problem (const mogs_string & filename);

        /** Solve the problem	 */
        void solve();


      private:
        SmartPtr < TNLP > nlp_;
        SmartPtr < IpoptApplication > app_;

};



#endif
