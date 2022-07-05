#ifndef CameraCriteria_HPP_
#define CameraCriteria_HPP_

#include "AbstractCriteria.h"

class CameraCriteria: virtual public AbstractCriteria
{
 public:
	CameraCriteria ();

    ~CameraCriteria ();

    virtual void init_from_xml( QDomElement criteria,
                        std::vector<MogsOptimDynamics<double> *>& dyns );

    virtual void init_from_AbstractCriteria(  AbstractCriteria* c);

    template<typename T>
      T compute( std::vector<MogsOptimDynamics<T> *>&dyns);

    double compute(std::vector<MogsOptimDynamics<double> *>&dyns)
    {
        return compute<double>(dyns);
    }

      private:

        double l1,l2;


        QString  desired_position_image,retine;
        QString  Position,Rotation,robot,Body,body_position;
        //   adouble su,sv,s,u,v;
        double  RotationX,RotationY,RotationZ,tval,erreur_projection;
        double focale,Kv,Ku,u0,v0;                           // Paramètres intrinsèques de la caméra

        Eigen::Matrix<double, 3,3> K;                //Matrice de rotation sur X d'un angle= pi/3 et matrice de calibration
        Eigen::Matrix<double, 3,1> Position_, Rotation_;           // correspond à la transaltion tx_ty_tz, supposant que le centre de l'object est à 1m du centre de la focale


        unsigned int nb_points_=0;
        // unsigned int body_id;
        std::vector< unsigned int> body_id_;

        Eigen::Matrix<double, 3,1> body_position_;
        Eigen::Matrix<double, 2,1> desired_position_image_;
        std::vector<Eigen::Matrix<double, 3,1>> bodyposition;
        std::vector<Eigen::Matrix<double, 2,1>> desiredpositionimage;

        std::vector<Eigen::Matrix<double, 3,1>> droite_point_;

        SpatialTransform<double> camera_pose_;

        SpatialTransform<double> M ;                 // Matrice_de_projection camera 1;


};
#include "CameraCriteria.hxx"
#endif // CameraCriteria_HPP_INCLUDED
