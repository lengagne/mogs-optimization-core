#include <TransformationConstraint.hpp>

TransformationConstraint::TransformationConstraint (  )
{
    plugin_name_ = "Transformation";
}

TransformationConstraint::TransformationConstraint(  std::vector<MogsOptimDynamics<double> *> &dyns,
                                                     const QString& robot_name1,
                                                     const QString& body_name1,
                                                     const SpatialTransform<double> & t1,
                                                     const QString& robot_name2,
                                                     const QString& body_name2,
                                                     const SpatialTransform<double> & t2,
                                                     const SpatialTransform<double> & t_diff):TransformationConstraint()
{
    Robot1 = robot_name1;
    Body1=body_name1;
    Robot2 = robot_name2;
    Body2=body_name2;
    init(dyns,Robot1,Body1,Robot2,Body2);
    Transformation_ = t1 * t_diff * t2.transpose();
}

void TransformationConstraint::init( std::vector<MogsOptimDynamics<double> *> &dyns,
                                     const QString& robot_name1,
                                     const QString& body_name1,
                                     const QString& robot_name2,
                                     const QString& body_name2 )
{
   robot1_id_ = -1;
    robot2_id_ = -1;
    for (int i=0;i<dyns.size();i++)        if(Robot1 == dyns[i]->getRobotName()){
        robot1_id_ = i;
        break;
    }
    for (int i=0;i<dyns.size();i++)        if(Robot2 == dyns[i]->getRobotName()){
        robot2_id_ = i;
        break;
    }

    if(robot1_id_==-1)
    {
        std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
        std::cerr<<"Error cannot recognize robot1_id for robot "<< Robot1.toStdString() <<std::endl;
        std::cerr<<"Known robots are : "<<std::endl;
        for (int i=0;i<dyns.size();i++)
            std::cerr<<"\t"<< dyns[i]->getRobotName().toStdString()  <<std::endl;
        exit(-1);
    }

    if(robot2_id_==-1)
    {
        std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
        std::cerr<<"Error cannot recognize robot2_id for robot "<< Robot2.toStdString() <<std::endl;
        std::cerr<<"Known robots are : "<<std::endl;
        for (int i=0;i<dyns.size();i++)
            std::cerr<<"\t"<< dyns[i]->getRobotName().toStdString()  <<std::endl;
        exit(-1);
    }


    body1_id_ = dyns[robot1_id_]->model->GetBodyId(Body1);
    if (body1_id_  ==  std::numeric_limits <unsigned int >::max () )
    {
        std::cout << "   Body_ (" <<  Body1.toStdString() <<") is unkown"<< std::endl;
        exit(0);
    }


    body2_id_ = dyns[robot2_id_]->model->GetBodyId(Body2);
    if (body2_id_  ==  std::numeric_limits <unsigned int >::max () )
    {
        std::cout << "   Body_ (" <<  Body2.toStdString() <<") is unkown"<< std::endl;
        exit(0);
    }

    m = 6; //desired_Transformation_.size();
    upper_.resize(m);
    lower_.resize(m);
    for (int i=0;i<m;i++)
    {
        upper_[i] = lower_[i] = 0.;
    }
}

void TransformationConstraint::init_from_xml(   QDomElement contraint,
                                            std::vector<MogsOptimDynamics<double> *>& dyns)
{
    QDomElement Elrobot1 = contraint.firstChildElement("robot1");
    if (Elrobot1.isNull())
    {
        std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
        std::cerr<<"You must specify a balise robot1 for constraint type: Transformation"<<std::endl;
        exit(0);
    }else
        Robot1 = Elrobot1.text().simplified();

    QDomElement Elrobot2 = contraint.firstChildElement("robot2");
    if (Elrobot2.isNull())
    {
        std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
        std::cerr<<"You must specify a balise robot2 for constraint type: Transformation"<<std::endl;
        exit(0);
    }else
        Robot2 = Elrobot2.text().simplified();

    QDomElement Elbody1 = contraint.firstChildElement("body1");
    if (Elbody1.isNull())
    {
        std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
        std::cerr<<"You must specify a balise body1 for constraint type: Transformation"<<std::endl;
        exit(0);
    }else
        Body1 = Elbody1.text().simplified();

    QDomElement Elbody2 = contraint.firstChildElement("body2");
    if (Elbody2.isNull())
    {
        std::cerr<<"Error in "<<__FILE__<<" at line "<< __LINE__<<std::endl;
        std::cerr<<"You must specify a balise body2 for constraint type: Transformation"<<std::endl;
        exit(0);
    }else
        Body2 = Elbody2.text().simplified();

    init(dyns,Robot1,Body1,Robot2,Body2);

    QDomElement Elposition1 = contraint.firstChildElement("position1");
    Eigen::Matrix<double,3,1> pos = convert_to_vec3(Elposition1);
    QDomElement Elrotation1 = contraint.firstChildElement("rotation1");
    Eigen::Matrix<double,3,1> rot = convert_to_vec3(Elrotation1);
    SpatialTransform<double> Transformation1 = SpatialTransform<double>(rot,pos);

    QDomElement Elposition2 = contraint.firstChildElement("position2");
    pos = convert_to_vec3(Elposition2);
    QDomElement Elrotation2 = contraint.firstChildElement("rotation2");
    rot = convert_to_vec3(Elrotation2);
    SpatialTransform<double> Transformation2 = SpatialTransform<double>(rot,pos);

    QDomElement Elposition = contraint.firstChildElement("position");
    pos = convert_to_vec3(Elposition);
    QDomElement Elrotation = contraint.firstChildElement("rotation");
    rot = convert_to_vec3(Elrotation);
    SpatialTransform<double> Transformation = SpatialTransform<double>(rot,pos);

    Transformation_ = Transformation2.transpose() * Transformation * Transformation1;
    std::cout<<"Transformation_ = "<<  Transformation_ <<std::endl;
}
   //

TransformationConstraint::~TransformationConstraint ()
{
}


void TransformationConstraint::init_from_AbstractConstraint(  AbstractConstraint* c)
{
    *this =  *(dynamic_cast<TransformationConstraint*>(c));
}
