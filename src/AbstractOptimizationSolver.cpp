//      MogsAbstractOptimization.h
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

#include "AbstractOptimizationSolver.h"

void AbstractOptimizationSolver::prepare()
{
    pb_->set_robots( robots_ );
    pb_->set_root(root_);
    pb_->load_xml();
}

void AbstractOptimizationSolver::set_root(QDomElement root)
{
    root_ = root;
    solver_xml_ =root_.firstChildElement("solver");
    if (solver_xml_.isNull())
    {
        std::cerr<<__FILE__<<" at line "<< __LINE__<<std::endl;
        std::cerr<<"Error the file must containt the balise \"solver\""<<std::endl;
        exit(0);
    }

    #ifdef MogsVisu_FOUND
    visu_during_optim_ = false;
    QDomElement ElVisuDuring=root_.firstChildElement("visu_during_optim");
    if (!ElVisuDuring.isNull())
    {
        visu_during_optim_ = convert_to_bool(ElVisuDuring.text().simplified());
        std::cout<<"visu_during_optim_ = "<< visu_during_optim_ <<std::endl;
    }
    #endif // MogsVisu_FOUND
    read_solver_option();
}
