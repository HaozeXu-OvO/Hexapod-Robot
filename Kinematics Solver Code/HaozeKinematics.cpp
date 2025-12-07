#include "HaozeKinematics.h"
#define N 4  

HaozeKinematics::HaozeKinematics(int num_of_joints_) {
    num_of_joints = num_of_joints_;
    num_of_joints_declared = 0;

}
//Establishing a robotic kinematic model using the MDH parameter method
void HaozeKinematics::AddJointMDH(float alpha, float a, float theta, float d) {
    Mdh[num_of_joints_declared][0] = alpha;
    Mdh[num_of_joints_declared][1] = a;
    Mdh[num_of_joints_declared][2] = theta;
    Mdh[num_of_joints_declared][3] = d;
    offset[num_of_joints_declared] = theta;

    num_of_joints_declared++;

}
//Establishing a robotic kinematic model through SDH parameter methods
void HaozeKinematics::AddJointSDH(float theta, float d, float alpha, float a) {
    Sdh[num_of_joints_declared][0] = theta;
    Sdh[num_of_joints_declared][1] = d;
    Sdh[num_of_joints_declared][2] = alpha;
    Sdh[num_of_joints_declared][3] = a;
    offset[num_of_joints_declared] = theta;
    
    num_of_joints_declared++;

}
//Solving robot kinematics using the MDH parameter method
void HaozeKinematics::forwardMDH() {
    // Serial.println("Start MDH 2 Transformer---------------");
    
    for(int i=0;i<num_of_joints;i++)
    {
        float alpha, a, theta, d;
        // Convert degrees to radians
        alpha = Mdh[i][0];
        a = Mdh[i][1];
        theta = Mdh[i][2] + JointState[i];
        d = Mdh[i][3];
        
        float alpha_rad = alpha;//radians(alpha);
        float theta_rad = theta;//radians(theta);
        //Define an intermediate variable
        mtx_type zhongjian[4][4] = {
        {cos(theta_rad), -sin(theta_rad), 0, a},
        {sin(theta_rad)* cos(alpha_rad), cos(theta_rad) * cos(alpha_rad), -sin(alpha_rad), -d * sin(alpha_rad)},
        {sin(theta_rad)* sin(alpha_rad), cos(theta_rad)* sin(alpha_rad), cos(alpha_rad), d * cos(alpha_rad)},
        {0, 0, 0, 1}
    };
    //Matrix.Copy((mtx_type*)A, N, N, (mtx_type*)B);B=A;Homogeneous matrix assignment
    Matrix.Copy((mtx_type*)zhongjian, N, N, (mtx_type*)MatrixO[i]);
    // Matrix.Print((mtx_type*)MatrixO[i], N, N, String(i+1));//Print the homogeneous matrix in the middle
    }
    // Serial.println("End MDH 2 Transformer---------------");

    //MatrixOb
    //C=A*B;
    //Multiply(mtx_type* A, mtx_type* B, int m, int p, int n, mtx_type* C)
    // Serial.println("Start Forward Kinematics calculate---------------");
    mtx_type zhongjian[4][4] = {
        {1.00 ,0.00 ,0.00 ,0.00},
        {0.00 ,1.00 ,0.00 ,0.00},
        {0.00 ,0.00 ,1.00 ,0.00},
        {0.00 ,0.00 ,0.00 ,1.00}
        };
    mtx_type jieguo[4][4] = {
        {1.00 ,0.00 ,0.00 ,0.00},
        {0.00 ,1.00 ,0.00 ,0.00},
        {0.00 ,0.00 ,1.00 ,0.00},
        {0.00 ,0.00 ,0.00 ,1.00}
        };
    for(int i=0;i<num_of_joints;i++)
    {
        Matrix.Multiply((mtx_type*)zhongjian, (mtx_type*)MatrixO[i], N, N, N, (mtx_type*)jieguo);

        Matrix.Copy((mtx_type*)jieguo, N, N, (mtx_type*)zhongjian);
        //Assign the final result to the matrixOb matrix.
        Matrix.Copy((mtx_type*)jieguo, N, N, (mtx_type*)MatrixOb[i]);
    }
    // Serial.println("End Forward Kinematics calculate---------------");

    for(int i=0;i<num_of_joints;i++)
    {
        JointPoint[i][0][0] = MatrixOb[i][0][3];
        JointPoint[i][1][0] = MatrixOb[i][1][3];
        JointPoint[i][2][0] = MatrixOb[i][2][3];
        JointPoint[i][3][0] = 1;
    }
    
    End_effectorPoint[0][0] = MatrixOb[num_of_joints-1][0][3];
    End_effectorPoint[1][0] = MatrixOb[num_of_joints-1][1][3];
    End_effectorPoint[2][0] = MatrixOb[num_of_joints-1][2][3];
    End_effectorPoint[3][0] = 1;

}

//Print MatrixO
void HaozeKinematics::PrintMatrixO() {
    Serial.println("Start Print MatrixO---------------");
    for(int i=0;i<num_of_joints;i++)
    {
        Matrix.Print((mtx_type*)MatrixO[i], N, N, String(i+1));
    }
    Serial.println("Ene Print MatrixO---------------");
}

//Print MatrixOb
void HaozeKinematics::PrintMatrixOb() {
    Serial.println("Start Print MatrixOb---------------");
    for(int i=0;i<num_of_joints;i++)
    {
        Matrix.Print((mtx_type*)MatrixOb[i], N, N, String(i+1));
    }
    Serial.println("Ene Print MatrixOb---------------");  
}

void HaozeKinematics::PrintEnd_effectorPoint() {
    char buffer[50]; // Adjust the size based on your needs
    Serial.print("End_effectorPoint:");
    Serial.print("|");
    snprintf(buffer, sizeof(buffer), " x: %-9.2f y: %-9.2f z: %-9.2f ", End_effectorPoint[0][0], End_effectorPoint[1][0], End_effectorPoint[2][0]);
    Serial.print(buffer);
    Serial.println(); 
}

