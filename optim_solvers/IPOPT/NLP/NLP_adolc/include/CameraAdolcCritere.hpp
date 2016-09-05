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


                    QString  desired_position_image,retine;
                    QString  Position,robot,Body,body_position;
                 //   adouble su,sv,s,u,v;
                    double  RotationX,RotationY,RotationZ,tval,erreur_projection;
                    double focale,Kv,Ku,u0,v0;                           // Paramètres intrinsèques de la caméra

                    Eigen::Matrix<double, 3, 3> RX,RY,RZ,K;                //Matrice de rotation sur X d'un angle= pi/3 et matrice de calibration
                    Eigen::Matrix<double, 3, 4> RT;                 //Matrice de rotation et de translation;
                    Eigen::Matrix<double, 3,1> Position_;           // correspond à la transaltion tx_ty_tz, supposant que le centre de l'object est à 1m du centre de la focale


                    int nb_points=0;
                   // unsigned int body_id;
                        std::vector< unsigned int> nb_points_;
                     std::vector< unsigned int> body_id_;

                     Eigen::Matrix<double, 3,1> body_position_;
                     Eigen::Matrix<double, 2,1> desired_position_image_;
                     std::vector<Eigen::Matrix<double, 3,1>> bodyposition;
                     std::vector<Eigen::Matrix<double, 2,1>> desiredpositionimage;



                    Eigen::Matrix<double, 3,4> M ;                 // Matrice_de_projection camera 1;


};
#include "CameraAdolcCritere.hxx"
#endif // CameraAdolcCritere_HPP_INCLUDED
