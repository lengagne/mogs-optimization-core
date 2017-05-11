#include <CloseToMiddleCriteria.hpp>


CloseToMiddleCriteria::CloseToMiddleCriteria (  QDomElement critere,
                                                    MogsKinematics<double>* kin)
{
    weight_ = critere.attribute("weight").toDouble();

    std::cout<<"weight = "<< weight_<<std::endl;
    QDomElement Child=critere.firstChildElement().toElement();
    while (!Child.isNull())
     {

         if (Child.tagName()=="robot")
           {
                Robot=Child.firstChild().toText().data();
                std::cout << "   Robot  = " << Robot.toStdString().c_str() << std::endl;
           }
       Child = Child.nextSibling().toElement();
    }

   // calcul de qm
    // a partir de kin->model
    std::vector<double> qmin;
    std::vector<double> qmax;
    kin->model->getPositionLimit(qmin,qmax);
    qm_.resize(kin->getNDof());
    for (int i=0; i<kin->getNDof(); i++)
    {
        qm_(i) = (qmin[i] + qmax[i])/2;
    }


}

CloseToMiddleCriteria::~CloseToMiddleCriteria ()
{
}





