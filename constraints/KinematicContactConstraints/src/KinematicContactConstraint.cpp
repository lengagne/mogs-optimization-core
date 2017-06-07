#include <KinematicContactConstraint.hpp>

KinematicContactConstraint::KinematicContactConstraint (  QDomElement ele,
                                       std::vector<MogsOptimDynamics<double> *>& dyns)
{
    qDebug()<<"Constructor of KinematicContactConstraint";
    m = 1;

    QDomElement Child=ele.firstChildElement().toElement();

    while (!Child.isNull())
    {
//         if (Child.tagName()=="robot")
//           {
//                Robot=Child.firstChild().toText().data();
//                std::cout << "   Robot  = " << Robot.toStdString().c_str() << std::endl;
//           }
       Child = Child.nextSibling().toElement();
    }
    qDebug()<<"m = "<< m ;

    upper.resize(1);
    upper(0) =radius_1_ + radius_2_;
    lower.resize(1);
    lower(0) =radius_1_ + radius_2_;

}

KinematicContactConstraint::~KinematicContactConstraint ()
{
}
