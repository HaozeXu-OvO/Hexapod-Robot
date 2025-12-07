/*******************************************************
   主板：Haoze_Servo8266
   功能：Haoze_H1mini六足机器人库文件
   引脚：SDA:21   SCL:22
   对于ARDUINO UNO，SCL:A5，SDA:A4
   Designer: Allen
   E-mail:953598974@qq.com
   Date:2024-08-14
*******************************************************/
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include "HaozeKinematics.h"
#include "MatrixMath.h"
// #include "Stepdata.h"

#define pi 3.14159
#define N 4  //矩阵阶数
#define pi4 0.78539816325  //pi/4

#define l1 34         //腿部连杆1
#define l2 43.51      //腿部连杆2
#define l3 93.07      //腿部连杆3

#define ll 94.93715   //身体斜线一半
#define lm 50.8       //身体中间横线一半

//#define jointnum 6    //机器人连杆数量


#ifdef servo180
//#define SERVOMIN  102               //0.5/20 * 4096 = 102
//#define SERVOMID  307               //1.5/20 * 4096 = 307
//#define SERVOMAX  512               //2.5/20 * 4096 = 512
//实际测试
#define SERVOMIN  102               
#define SERVOMID  327               
#define SERVOMAX  552
#endif

#define SERVOMIN  102               
#define SERVOMID  327               
#define SERVOMAX  552




class Haoze_H1mini
{
    public:
    int speed = 10;              //速度控制参数
    int stepheight = 1;          //身高模式1，2，3
    float stepx = 5;             //x方向步长(左右走)
    float stepy = 5;             //y方向步长(前后走)
    float stepz = 5;             //z方向步长(抬脚高度)
    int forward_delay = 2;       //前进速度控制参数
    int backward_delay = 2;      //后退速度控制参数
    int leftward_delay = 5;      //左转速度控制参数
    int rightward_delay = 5;     //右转速度控制参数
    float zhuanwanjiaodu = 0.6981;//30度转弧度：0.5236；40度转弧度：0.6981
    float zcali = 3.0;
    float baseanglex = 40.0;     //机身倾角
    float baseangley = 40.0;
    float baseanglez = 50.0;
    float theta_rh[3];          //六足机器人单腿逆解三个关节变量暂存数组


    //leg根部到foot的逆矩阵
    mtx_type Base_footM[6][4][4];
    mtx_type Base_legM[6][4][4];
    mtx_type Leg_footM[6][4][4];

    //Endpose_3,这个点是foot相对于leg根部的坐标
    float Endpose_3[6][4][1]=
    {
    0.0 ,0.0 ,0.0 , 1.0,
    0.0 ,0.0 ,0.0 , 1.0,
    0.0 ,0.0 ,0.0 , 1.0,
    0.0 ,0.0 ,0.0 , 1.0,
    0.0 ,0.0 ,0.0 , 1.0,
    0.0 ,0.0 ,0.0 , 1.0
    };

    //机器人关节舵机接口号
    byte Jointservo[18] = {
        0,1,2,
        3,4,5,
        6,7,8,
        9,10,11,
        12,13,14,
        15,16,17};
    //机器人关节标定量
    signed char rec[18] = {
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0};
    //机器人当前关节变量，舵机当前角度
    float Servo_p[18] = {
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 };
    //机器人目标关节变量，舵机目标角度
    float Servo_r[18] = {
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0};
        //机器人当前关节变量
    float Joint_P[18] = {
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0};
    //机器人期望关节变量
    float Joint_R[18] = {
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0};
    //机器人关节变量0
    float Servo0[18] = {
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0};
    float joint_s[18]={0.0,30.0,-30.0,0.0,30.0,-30.0,0.0,30.0,-30.0,0.0,30.0,-30.0,0.0,30.0,-30.0,0.0,30.0,-30.0};
    //舵机方向反相参数
    int direct[18] = {
        1,-1,1,
        1,-1,1,
        1,-1,1,
        1,-1,1,
        1,-1,1,
        1,-1,1};

    Adafruit_PWMServoDriver pwm;
    Adafruit_PWMServoDriver pwm1;

    //六条腿的运动学模型(MDH参数法)
    HaozeKinematics kinRmLeg;
    HaozeKinematics kinRfLeg;
    HaozeKinematics kinLfLeg;
    HaozeKinematics kinLmLeg;
    HaozeKinematics kinLbLeg;
    HaozeKinematics kinRbLeg;

    Haoze_H1mini()
        : kinRmLeg(6), kinRfLeg(6), kinLfLeg(6), kinLmLeg(6), kinLbLeg(6), kinRbLeg(6) {
        // 可以在这里进行其他初始化操作
    }

    void init();
    void printcaliData();
    void connect();
    void servowrite(unsigned int id,float angle);
    void jointwrite(unsigned int id,float angle);
    void framewrite(float angle[18]);
    void forward(unsigned int step);
    void backward(unsigned int step);
    void turnleft(unsigned int step);
    void turnright(unsigned int step);
    void leftward(unsigned int step);
    void rightward(unsigned int step);
    void frame2frame(float angle_p[18],float angle_r[18]);

    void SetStartJointstate();//初始化关节变量
    void ForwardKinematics_GetBasefootM();//正运动学然后获取Basefoot矩阵
    void Forward_GetBaselegMsv();
    void GetLegfootM();
    void ik4(float zuobiao[3]);
    void GetLegfootpose_IK2H1mini_Jointstate();

    void printLegFootpose();
    void StartForward();//前进起步
    void forward_y(float ylength);//机器人前进函数,参数ylength为前进距离毫米
    void EndForward();//前进收腿

    void StartBackward();
    void EndBackward();
    void backward_y(float ylength);//这个函数控制机器人后退多少距离

    void StartRightward();//右移起步
    void EndRightward();//右移收腿
    void rightward_x(float ylength);//这个函数控制机器人右移多少距离

    void StartLeftward();
    void EndLeftward();
    void leftward_x(float ylength);//这个函数控制机器人左移多少距离

    void start_turnleft();
    void end_turnleft();
    void turnleft_angle(float angleleft);
    void turnleft_angle1(float angleleft);
    void turnleft_angle2(float angleleft);

    void start_turnright();
    void end_turnright();
    void turnright_angle(float angleright);
    void turnright_angle1(float angleright);
    void turnright_angle2(float angleright);
    

};

class Haoze_Q1mini
{
    public:
    int speed = 10;
    //机器人关节标定量
    signed char rec[12] = {
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0};
    //机器人当前关节变量，舵机当前角度
    float Servo_p[12] = {
        0.0 , 0.0 , 0.0,
        0.0 , 0.0 , 0.0,
        0.0 , 0.0 , 0.0,
        0.0 , 0.0 , 0.0};
    //机器人目标关节变量，舵机目标角度
    float Servo_r[12] = {
        0.0 , 0.0 , 0.0,
        0.0 , 0.0 , 0.0,
        0.0 , 0.0 , 0.0,
        0.0 , 0.0 , 0.0};
    //舵机方向反相参数
    int direct[12] = {
        1,1,-1,
        1,1,-1,
        1,-1,1,
        1,-1,1};

    Adafruit_PWMServoDriver pwm;
    Adafruit_PWMServoDriver pwm1;
    
    void init();
    void Print_EEPROM();
    void connect();
    void servowrite(unsigned int id,float angle);
    void framewrite(float angle[12]);
    void forward(unsigned int step);
    void backward(unsigned int step);
    void turnleft(unsigned int step);
    void turnright(unsigned int step);
    void leftward(unsigned int step);
    void rightward(unsigned int step);
    void flame2frame(float angle_p[12],float angle_r[12]);
    

};

class Haoze_Armini4
{
    public:
    int speed = 100;
    byte Jointservo[4] = {
        18,19,20,
        21};
    //机器人关节标定量
    signed char rec[4] = {
        0,0,0,0};
    //机器人当前关节变量，舵机当前角度
    float Servo_p[4] = {
        0.0 , 0.0 , 0.0 ,
        0.0 };
    //机器人目标关节变量，舵机目标角度
    float Servo_r[4] = {
        0.0 , 0.0 , 0.0 ,
        0.0 };
    //舵机方向反相参数
    int direct[4] = {
        1,1,-1,1};

    Adafruit_PWMServoDriver pwm;
    Adafruit_PWMServoDriver pwm1;
    
    void init();
    void connect();
    float bezierBlend(float t, float p0, float p1, float p2, float p3);
    float smoothSplineMove(float startAngle, float endAngle, int steps, int i);
    void servowrite(unsigned int id,float angle);
    void jointwrite(unsigned int id,float angle);
    void framewrite(float angle[4]);
    void forward(unsigned int step);
    void backward(unsigned int step);
    void turnleft(unsigned int step);
    void turnright(unsigned int step);
    void leftward(unsigned int step);
    void rightward(unsigned int step);
    void flame2frame(float angle_p[4],float angle_r[4]);
    

};

class Haoze_Spider8
{
    public:
    int speed = 10;
    //机器人关节标定量
    signed char rec[18] = {
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0};
    //机器人当前关节变量，舵机当前角度
    float Servo_p[18] = {
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 };
    //机器人目标关节变量，舵机目标角度
    float Servo_r[18] = {
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0};
    //舵机方向反相参数
    int direct[18] = {
        -1,1,1,
        -1,1,1,
        -1,1,1,
        -1,1,1,
        -1,1,1,
        -1,1,1};

    Adafruit_PWMServoDriver pwm;
    Adafruit_PWMServoDriver pwm1;
    
    void init();
    void connect();
    void servowrite(unsigned int id,float angle);
    void framewrite(float angle[18]);
    void forward(unsigned int step);
    void backward(unsigned int step);
    void turnleft(unsigned int step);
    void turnright(unsigned int step);
    void leftward(unsigned int step);
    void rightward(unsigned int step);
    void flame2frame(float angle_p[18],float angle_r[18]);
    

};

class Haoze_Spider12
{
    public:
    int speed = 10;

    //机器人关节标定量
    signed char rec[18] = {
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0};
    //机器人当前关节变量，舵机当前角度
    float Servo_p[18] = {
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 };
    //机器人目标关节变量，舵机目标角度
    float Servo_r[18] = {
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0 ,
        0.0 , 0.0 , 0.0};
    //舵机方向反相参数
    int direct[18] = {
        -1,1,1,
        -1,1,1,
        -1,1,1,
        -1,1,1,
        -1,1,1,
        -1,1,1};

    Adafruit_PWMServoDriver pwm;
    Adafruit_PWMServoDriver pwm1;
    
    void init();
    void Print_EEPROM();
    void connect();
    void servowrite(unsigned int id,float angle);
    
    void framewrite(float angle[18]);
    void forward(unsigned int step);
    void backward(unsigned int step);
    void turnleft(unsigned int step);
    void turnright(unsigned int step);
    void leftward(unsigned int step);
    void rightward(unsigned int step);
    void flame2frame(float angle_p[18],float angle_r[18]);
    

};