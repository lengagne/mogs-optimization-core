#ifndef CameraAdolcCritere_HPP_
#define CameraAdolcCritere_HPP_

#include <NLP_adolc.hpp>
#include <adolc.h>
#include "AbstractAdolcCritere.hpp"
#include "MogsNlpIpopt.hpp"




class CameraAdolcCritere: public AbstractAdolcCritere
{
 public:
	CameraAdolcCritere (QDomElement critere,
                           MogsKinematics<double> *kin);

    ~CameraAdolcCritere ();

    double compute( const double *x , MogsKinematics<double> * kin)
    {
        return compute<double>(x,kin);
    }

    adouble compute( const adouble *x , MogsKinematics<adouble> * kin)
    {
        return compute<adouble>(x,kin);
    }

    template<typename T>
      T compute( const T *x,MogsKinematics<T> *kin_);





      private:

                     double l1,l2;


                    QString  desired_position_image,name;
                    QString  Position,robot,Body,body_position;
                 //   adouble su,sv,s,u,v;
                    double  RotationX,tval,erreur_projection;
                    double focale,alpha_v,alpha_u,u0,v0;            // Paramètres intrinsèques de la caméra

                    Eigen::Matrix<double, 3, 3> R,K;                //Matrice de rotation sur X d'un angle= pi/3 et matrice de calibration
                    Eigen::Matrix<double, 3, 4> RT;                 //Matrice de rotation et de translation;
                    Eigen::Matrix<double, 3,1> Position_;           // correspond à la transaltion tx_ty_tz, supposant que le centre de l'object est à 1m du centre de la focale


                    unsigned int nb_points_;
                    unsigned int body_id_;
                    Eigen::Matrix<double, 3,1> body_position_;
                    Eigen::Matrix<double, 2,1> desired_position_image_;

                    Eigen::Matrix<double, 3,4> M ;                 // Matrice_de_projection camera 1;


};
#include "CameraAdolcCritere.hxx"
#endif // CameraAdolcCritere_HPP_INCLUDED
