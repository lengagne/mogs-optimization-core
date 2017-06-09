#include <ToZeroConstraint.hpp>

ToZeroConstraint::ToZeroConstraint (  QDomElement ele,
                                       std::vector<MogsOptimDynamics<double> *>& dyns)
{
    qDebug()<<"Constructor of ToZeroConstraint";
    m = 1;
    n = 0;
    for (int i=0;i<dyns.size();i++)
        n += dyns[i]->getNDof();
    upper_.push_back(0);
    lower_.push_back(0);

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

ToZeroConstraint::~ToZeroConstraint ()
{
}
