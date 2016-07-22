#include <PositionAdolcCritere.hpp>


PositionAdolcCritere::PositionAdolcCritere ()
{

}

PositionAdolcCritere::~PositionAdolcCritere ()
{
}




/*
void PositionAdolcCritere::read_problem (const mogs_string & filename)
{

 MogsAbstractProblem::read_problem(filename);

   QDomElement criteres=root_.firstChildElement("criteres");

    std::cout<<"find criteres = "<< !criteres.isNull()<<std::endl;

    for (QDomElement critere = criteres.firstChildElement ("critere"); !critere.isNull(); critere = critere.nextSiblingElement("critere"))
	{

 while(!critere.isNull())
{
//        std::cout<<"find one critere"<<std::endl;
//        QString Type=critere.attribute("Type","position");
//        QString  weight=critere.attribute(" weight","1");
//        std::cout << "critere type = " << Type.toStdString().c_str() << std::endl;
//        std::cout << "critere weight = " << weight.toStdString().c_str() << std::endl;

        // Get the first child of the component
        QDomElement Child=critere.firstChildElement().toElement();
        QString Body;
        QString Robot;
        QString body_position;
        QString desired_position;
            // Read Name and value

        while (!Child.isNull())
        {
              if (Child.tagName()=="robot")
            {
            Robot=Child.firstChild().toText().data();
            std::cout << "   Robot  = " << Robot.toStdString().c_str() << std::endl;
            }

             if (Child.tagName()=="body")
            {
            Body=Child.firstChild().toText().data();
            std::cout << "   Body  = " << Body.toStdString().c_str() << std::endl;
            }
            if (Child.tagName()=="body_position")
            {body_position=Child.firstChild().toText().data();
            std::cout << "   body_position  = " << body_position.toStdString().c_str() << std::endl;
            }
             if (Child.tagName()=="desired_position")
            {desired_position=Child.firstChild().toText().data();
             std::cout << "   desired_position  = " << desired_position.toStdString().c_str() << std::endl;
            }
           Child = Child.nextSibling().toElement();
        }

        critere = critere.nextSibling().toElement();


  }  }
*/



