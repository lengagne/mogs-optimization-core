#include <KinematicContactConstraint.hpp>

KinematicContactConstraint::KinematicContactConstraint (  QDomElement ele,
                                       std::vector<MogsOptimDynamics<double> *>& dyns)
{
    qDebug()<<"Constructor of KinematicContactConstraint";
    m = 1;
    n = 0;
    for (int i=0;i<dyns.size();i++)
        n += dyns[i]->getNDof();
    upper.resize(1);
    upper(0) =0;
    lower.resize(1);
    lower(0) =0;

    QDomElement Child=ele.firstChildElement().toElement();

    while (!Child.isNull())
    {

         if (Child.tagName()=="robot")
           {
                Robot=Child.firstChild().toText().data();
                std::cout << "   Robot  = " << Robot.toStdString().c_str() << std::endl;
           }
       Child = Child.nextSibling().toElement();
    }
    qDebug()<<"m = "<< m ;
}

KinematicContactConstraint::~KinematicContactConstraint ()
{
}
