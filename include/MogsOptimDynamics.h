/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 * Code Modified by S. Lengagne during his work in the following labs:
 *      2011-2012:  Karlsruhe Institute fur Technologie, Karlsruhe, Germany
 *      2013-2012: IUT de Beziers/ LIRMM, Beziers, France
 * 	from 2013 : Université Blaise Pascal / axis : ISPR / theme MACCS
 */

#ifndef _MOGS_OPTIM_DYNAMICS_H
#define _MOGS_OPTIM_DYNAMICS_H

#include <assert.h>
#include <iostream>

#include "MogsDynamics.h"

namespace RigidBodyDynamics
{

	template < typename T > class MogsOptimDynamics : virtual public MogsDynamics < T >
	{
	      public:
		MogsOptimDynamics ()
		{

		};

		MogsOptimDynamics (MogsRobotProperties* Robot_in)
		{
			SetRobot(Robot_in);
		};

		~MogsOptimDynamics ()
		{
		};

		void reset_forces()
		{
		    for (int i=0;i<MogsDynamics < T >::getNBodies();i++)
                f_ext_[i] =  Eigen::Matrix < T, 6, 1 >::Zero();
		}

		void SetRobot(MogsRobotProperties* Robot_in)
		{
		    MogsDynamics < T >::SetRobot(Robot_in);
		    q_ = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero( MogsDynamics < T >::getNDof());
		    dq_ = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero( MogsDynamics < T >::getNDof());
		    ddq_ =Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero( MogsDynamics < T >::getNDof());
		    tau_ = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero( MogsDynamics < T >::getNDof());
		    q_ = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero(MogsDynamics < T >::getNDof());
		    dq_ = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero(MogsDynamics < T >::getNDof());
		    ddq_ = Eigen::Matrix < T, Eigen::Dynamic, 1 >::Zero(MogsDynamics < T >::getNDof());
		    f_ext_.resize(MogsDynamics < T >::getNBodies());
            reset_forces();
		}

        void UpdateStaticDynamics()
        {
            // FIXME modif inverse dynamics to do inverse static
            MogsDynamics < T >::InverseDynamics(q_,dq_,ddq_,tau_,&f_ext_);
        }


         Eigen::Matrix < T, Eigen::Dynamic, 1 > q_, dq_, ddq_, tau_;

         std::vector < Eigen::Matrix < T, 6, 1 >, Eigen::aligned_allocator < Eigen::Matrix < T, 6, 1 > > > f_ext_;

	};


}

#endif /* _MOGS_OPTIM_DYNAMICS_H */
