#include <ToZeroConstraint.hpp>

ToZeroConstraint::ToZeroConstraint (  QDomElement ele,
                                                    MogsKinematics<double>* kin)
{
    qDebug()<<"Constructor of ToZeroConstraint";
    m = 1;
    n = kin->getNDof();


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
    //
}

ToZeroConstraint::~ToZeroConstraint ()
{
}
