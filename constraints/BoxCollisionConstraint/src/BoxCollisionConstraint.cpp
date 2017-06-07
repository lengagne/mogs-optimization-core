#include <BoxCollisionConstraint.hpp>

BoxCollisionConstraint::BoxCollisionConstraint (  QDomElement ele,
                                                std::vector<MogsOptimDynamics<double> *>& dyns)
{
    m = 1;
//    qDebug()<<"Constructor of BoxCollisionConstraint";
    QDomElement ElRobot1=ele.firstChildElement("robot1");
    QDomElement ElRobot2=ele.firstChildElement("robot2");

    QDomElement ElBody1=ele.firstChildElement("body1");
    QDomElement ElBody2=ele.firstChildElement("body2");

    if(ElRobot1.isNull())
    {
        qDebug()<<"Error in constraint BoxCollisionConstraint tag \"robot1\" not found";
        exit(1);
    }
    if(ElRobot2.isNull())
    {
        qDebug()<<"Error in constraint BoxCollisionConstraint tag \"robot2\" not found";
        exit(1);
    }
    if(ElBody1.isNull())
    {
        qDebug()<<"Error in constraint BoxCollisionConstraint tag \"body1\" not found";
        exit(1);
    }
    if(ElBody2.isNull())
    {
        qDebug()<<"Error in constraint BoxCollisionConstraint tag \"body2\" not found";
        exit(1);
    }

    QString r1 = ElRobot1.firstChildElement("name").text().simplified();
    QString r2 = ElRobot2.firstChildElement("name").text().simplified();
    QString config1 = ElRobot1.firstChildElement("config_file").text().simplified();
    QString config2 = ElRobot2.firstChildElement("config_file").text().simplified();

    QString b1 = ElBody1.text().simplified();
    QString b2 = ElBody2.text().simplified();

    int nb = dyns.size();
    for (int i=0;i<nb;i++)
    {
        if( dyns[i]->getRobotName() == r1)
        {
            coll_.robot_1 = i;
            coll_.body_1 = dyns[i]->model->GetBodyId(b1);
            break;
        }
    }
    for (int i=0;i<nb;i++)
    {
        if( dyns[i]->getRobotName() == r2)
        {
            coll_.robot_2 = i;
            coll_.body_2 = dyns[i]->model->GetBodyId(b2);
            break;
        }
    }


    QDomElement Eltype=ele.firstChildElement("type");
    upper.resize(1);
    lower.resize(1);
    upper(0) = lower(0) =0;
    if (!Eltype.isNull())
    {
        if (Eltype.text().simplified()=="zero")
        {

        }else if(Eltype.text().simplified()=="avoid")
        {
            lower(0) = 0.001;
            upper(0) = 1e10;
        }
        else if(Eltype.text().simplified()=="penetration")
            lower(0) = -1e10;
    }

//    std::vector<collision_value> tmp_colls;
//    tmp_colls.push_back(coll_);
//     // cast of the vector
//     std::vector<MogsKinematics<double> *> kins;
     for (int i=0;i<dyns.size();i++)
        dyns_.push_back((MogsKinematics<double>*)dyns[i]);
//    collision_computation_ = new MogsBoxCollision(kins,tmp_colls);
//    dyns_ = dyns;

    d1_ = new MogsBoxCollisionDefinition(mogs_get_absolute_link(config1),b1);
    d2_ = new MogsBoxCollisionDefinition(mogs_get_absolute_link(config2),b2);

    coll_detector_ = new MogsBoxCollision();
}

BoxCollisionConstraint::~BoxCollisionConstraint ()
{
    delete coll_detector_;
    delete d1_;
    delete d2_;
}

void BoxCollisionConstraint::compute(double * g, std::vector<MogsOptimDynamics<double> *>& dyns)
{
    SpatialTransform<double> T1,T2;
    dyns[coll_.robot_1]->getFrameCoordinate(coll_.body_1,T1);
    dyns[coll_.robot_2]->getFrameCoordinate(coll_.body_2,T2);
    coll_detector_->compute_one_distance<double>(T1,T2,coll_,d1_,d2_);

    g[offset] = coll_.distance;
}

