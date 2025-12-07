#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
// #include "Stepdata.h"

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
    int speed = 10;
    int stepheight = 1;
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
    //机器人目标关节变量，舵机目标角度
    float Servo0[4] = {
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