#include <ToZeroConstraint.hpp>

ToZeroConstraint::ToZeroConstraint ( )
{
    plugin_name_ = "Balance";
}

void ToZeroConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    *this =  *(dynamic_cast<ToZeroConstraint*>(c));
}

void ToZeroConstraint::init_from_xml (  QDomElement ele,
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

extern "C" ToZeroConstraint* create()
{
    return new ToZeroConstraint( );
}

extern "C" void destroy(ToZeroConstraint* p)
{
    delete p;
}