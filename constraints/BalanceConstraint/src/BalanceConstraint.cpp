#include <BalanceConstraint.hpp>

BalanceConstraint::BalanceConstraint ()
{
    plugin_name_ = "Balance";
}
BalanceConstraint::BalanceConstraint(  std::vector<MogsOptimDynamics<double> *> &dyns):BalanceConstraint()
{
    m=0;
    for (int i=0;i<dyns.size();i++)
        if( dyns[i]->model->is_robot_floating_base())
            m+=6;
    for (int i=0;i<m;i++)
    {
        upper_.push_back(0);
        lower_.push_back(0);
    }

//    Child = Child.nextSibling().toElement();
//   Body=Child.firstChild().toText().data().simplified();
}
BalanceConstraint::~BalanceConstraint ()
{

}

void BalanceConstraint::init_from_xml( QDomElement ele,
                                       std::vector<MogsOptimDynamics<double> *>& dyns)
{
//     qDebug()<<"Constructor of BalanceConstraint";
    m=0;
    for (int i=0;i<dyns.size();i++)
        if( dyns[i]->model->is_robot_floating_base())
            m+=6;
    for (int i=0;i<m;i++)
    {
        upper_.push_back(0);
        lower_.push_back(0);
    }
}

void BalanceConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    *this =  *(dynamic_cast<BalanceConstraint*>(c));
}

#ifdef MogsVisu_FOUND
void BalanceConstraint::update_visu (VisuHolder *visu,
                          std::vector<MogsOptimDynamics<double> *> & dyns,
                          const double * param)
{
    Eigen::Matrix<double,6,1> line;
    std::vector<Eigen::Matrix<double,6,1>> lines;
    Eigen::Matrix<double,3,1> COM;
    for (int i=0;i<dyns.size();i++)
        if( dyns[i]->model->is_robot_floating_base())
        {
//            for (unsigned int ii = dyns[i]->model->mBodies.size () - 1; ii > 0; ii--)
//            {
//                std::cout<<"dyns["<<i<<"]->model->S["<<ii<<"] = "<< dyns[i]->model->S[ii].transpose()<<std::endl;
//                for(int k=0;k<6;k++)	std::cout<<"dyns["<<i<<"]->model->f["<<ii<<"]("<<k<<") = "<< dyns[i]->model->f[ii](k)<<std::endl;
//                for(int k=0;k<6;k++)	std::cout<<"dyns["<<i<<"]->model->fext["<<ii<<"]("<<k<<") = "<< dyns[i]->f_ext_[ii](k)<<std::endl;
//            }
//
//            std::cout<<"q = "<< dyns[i]->q_.transpose()<<std::endl;
//             std::cout<<"tau = "<< dyns[i]->tau_.transpose()<<std::endl;
//             std::cout<<"nb_bodies("<<i<<") = "<< dyns[i]->getNBodies()<<std::endl;
            COM = dyns[i]->getCenterOfMAss();
//            std::cout<<"q = "<< dyns[i]->q_.transpose()<<std::endl;
//            std::cout<<"COM = "<< COM.transpose()<<std::endl;

            for (int k=0;k<3;k++)
            {
                line(k) = COM(k);
                line(k+3) = COM(k);
            }
            line(0) += 1;
            line(3) -= 1;
            lines.push_back(line);
            for (int k=0;k<3;k++)
            {
                line(k) = COM(k);
                line(k+3) = COM(k);
            }
            line(1) += 1;
            line(4) -= 1;
            lines.push_back(line);
            for (int k=0;k<3;k++)
            {
                line(k) = COM(k);
                line(k+3) = COM(k);
            }
            line(2) += 1;
            line(5) -= 1;
            lines.push_back(line);
        }
    visu->draw_additional_lines(lines,0,255,0);
}
#endif // MogsVisu_FOUND
