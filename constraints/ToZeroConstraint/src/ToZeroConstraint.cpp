#include <ToZeroConstraint.hpp>

ToZeroConstraint::ToZeroConstraint (  QDomElement ele,
                                                    MogsKinematics<double>* kin)
{
    qDebug()<<"Constructor ofToZeroConstraint";
    m = 1;
    n = kin->getNDof();
    double upper =0;
    double lower =0;

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
