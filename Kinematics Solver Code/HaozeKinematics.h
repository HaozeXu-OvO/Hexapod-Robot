#ifndef HaozeKinematic_H
#define HaozeKinematic_H

#include <Arduino.h>
#include "MatrixMath.h"

class HaozeKinematics {
    private:
        int num_of_joints;                          //关节总数
        int num_of_joints_declared;                 //当前关节号
        float Sdh[10][4];                           //SDH参数
        float offset[10];                           //关节变量偏置值
        mtx_type MatrixO[10][4][4];                 //相邻连杆齐次变换
        
        mtx_type mat;

    public:
        float Mdh[10][4];                           //MDH参数
        mtx_type MatrixOb[10][4][4];                //连杆基于base_link齐次变换
        float JointState[10];                       //关节变量
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