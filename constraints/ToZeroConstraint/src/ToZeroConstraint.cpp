#include <ToZeroConstraint.hpp>

ToZeroConstraint::ToZeroConstraint (  QDomElement ele,
                                                    MogsKinematics<double>* kin)
{
    qDebug()<<"Constructor of ToZeroConstraint";
    m = 1;
    n = kin->getNDof();
//    nnz_jac_g = kin->getNDof();
    lower.resize(m);
    upper.resize(m);
    upper(0) = 0;
    lower(0) = 0;
    //upper(1) = 0;
    //lower(1) = 0;

//    weight_ = ele.attribute("weight").toDouble();
//    std::cout<<"weight = "<< weight_<<std::endl;
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



//bool ToZeroConstraint::eval_g(Index n, Index m, const Number * x, Number * g)
//{
//// Contrainte 1 :
//    g[0] = 0.;
//    for (int i=0; i<n; i++)
//    {
//        g[0] += x[i];
//    }
//// Contrainte 2 :
////    g[1] = 0.;
////    for (int i=0; i<n; i++)
////    {
////        g[1] += x[i*2];
////    }
//    return true;
//}
//
//void ToZeroConstraint::particuliar_jacobian(Index * iRow, Index * jCol, Index n)
//{
//    for(j = 0; j < m; j++)
//        for (int i=0; i<n; i++)
//        {
//            iRow[i] = j;
//            jCol[i] = i;
//        }
//}
//
//void ToZeroConstraint::derivative(Number * values, Index n)
//{
//    for (int i=0; i<n; i++)
//    {
//        values[i] = 1.0;
//    }
//}

ToZeroConstraint::~ToZeroConstraint ()
{
}
