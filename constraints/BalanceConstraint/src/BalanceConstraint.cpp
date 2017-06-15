#include <BalanceConstraint.hpp>

BalanceConstraint::BalanceConstraint (  QDomElement ele,
                                       std::vector<MogsOptimDynamics<double> *>& dyns)
{
    qDebug()<<"Constructor of BalanceConstraint";
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

BalanceConstraint::~BalanceConstraint ()
{
}

#ifdef MogsVisu_FOUND
void BalanceConstraint::update_visu (VisuHolder *visu,
                          std::vector<MogsOptimDynamics<double> *> & dyns,
                          const double * param)
{
    Eigen::Matrix<double,6,1> line;
    std::vector<Eigen::Matrix<double,6,1>> lines;
    SpatialTransform<double> T ;
    Eigen::Matrix<double,3,1> COM;
    for (int i=0;i<dyns.size();i++)
        if( dyns[i]->model->is_robot_floating_base())
        {
            dyns[i]->getFrameCoordinate(6,T);
            std::cout<<"T = "<< T<<std::endl;
            COM = dyns[i]->getCenterOfMAss();
            for (int k=0;k<3;k++)
            {
                line(k) = COM(k);
                line(k+3) = COM(k);
            }
            line(2) += 10;
            line(5) -= 10;
            lines.push_back(line);
        }
    visu->draw_additional_lines(lines,0,255,0);
}
#endif // MogsVisu_FOUND
