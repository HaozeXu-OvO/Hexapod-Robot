#ifndef HaozeKinematic_H
#define HaozeKinematic_H

#include <Arduino.h>
#include "MatrixMath.h"

class HaozeKinematics {
    private:
        int num_of_joints;                          
        int num_of_joints_declared;                
        float Sdh[10][4];                           
        float offset[10];                           //Joint variable offset value
        mtx_type MatrixO[10][4][4];                 //Homogeneous transformation of adjacent links
        
        mtx_type mat;

    public:
        float Mdh[10][4];                           //MDH参数
        mtx_type MatrixOb[10][4][4];                //Linkage based on the homogeneous transformation of base_link
        float JointState[10];                       //Joint variable
        float JointPoint[10][4][1];
        float End_effectorPoint[4][1];
        HaozeKinematics(int num_of_joints_);
        //MDH Parameter:alpha , a , theta , d
        void AddJointMDH(float alpha, float a, float theta, float d);
        //MDH Parameter:theta , d , alpha , a
        void AddJointSDH(float theta, float d, float alpha, float a);

        void forwardMDH();
        void PrintMatrixO();
        void PrintMatrixOb();
        void PrintEnd_effectorPoint();

};

#endif