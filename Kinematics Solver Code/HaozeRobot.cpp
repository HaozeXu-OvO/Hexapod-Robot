#include "Arduino.h"
#include "HaozeRobot.h"
#include "Stepdata.h"
#include <EEPROM.h>  //引入库文件
//EEProm地址起始位
#define H1miniES 0           //六足机器人舵机标定
#define H1miniEF 4094        //六足机器人标定标志
#define Q1miniES 100         //四足机器人舵机标定
#define Q1miniEF 4094         //四足机器人标定标志
#define Armini4ES 200        //机械臂舵机标定
#define Armini4EF 399        //机械臂标定标志



void Haoze_H1mini::init(){
  Serial.begin(115200);
  Serial.println();
  Serial.println("Haoze_H1mini HexapodRobot program!");

  pwm = Adafruit_PWMServoDriver();               //驱动1~16或(0~15)号舵机
  pwm1 = Adafruit_PWMServoDriver(0x41);          //驱动17~32或(16~31)号舵机

  Wire.begin();//开启IIC通信
  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  signed char rec_lin;
  //存储下来0度的偏移量
  EEPROM.begin(4095);//初始化eeprom大小

  //判断eeprom是否经过写入
  rec_lin = EEPROM.read(H1miniEF);//最后面的这一个代表初始化标志,127为已初始化，否则为未初始化
  delay(1);
  if(rec_lin != 127)
  {
    Serial.println("First startup this Program!");
    Serial.println("Write EEPROM...");
    for(int i=0;i<18;i++)
    {
      EEPROM.write(H1miniES+i,0);
      EEPROM.commit();  
      delay(1);    
    }
    EEPROM.write(H1miniEF,127);
    EEPROM.commit();  
    Serial.println("Eeprom write done!");
  }
  else
  {
    //打印出来eeprom区域的所有数据
    for(int i=0;i<18;i++)
    {
      rec_lin = EEPROM.read(H1miniES+i);
      rec[i] = rec_lin;
      // Serial.print(rec[i]);Serial.print(",");
      delay(1);
    }
    Serial.println();
    Serial.println("The following is the Calibrate data :");
    Serial.println("+---+---+---+---+---+---+---+---+---+---+----+----+----+----+----+----+----+----+");
    Serial.println("| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 |");
    Serial.println("+---+---+---+---+---+---+---+---+---+---+----+----+----+----+----+----+----+----+");
  
    char buffer[200]; // Adjust the size based on your needs
    snprintf(buffer, sizeof(buffer), "|%-3d|%-3d|%-3d|%-3d|%-3d|%-3d|%-3d|%-3d|%-3d|%-3d| %-3d| %-3d| %-3d| %-3d| %-3d| %-3d| %-3d| %-3d|", rec[0] , rec[1], rec[2], rec[3], rec[4], rec[5], rec[6], rec[7], rec[8], rec[9], rec[10], rec[11], rec[12], rec[13], rec[14], rec[15], rec[16], rec[17]);
    Serial.println(buffer);
    Serial.println("+---+---+---+---+---+---+---+---+---+---+----+----+----+----+----+----+----+----+");
  
    Serial.println();
    Serial.println("Eeprom Read done!");
  }


  //初始化六条腿的MDH参数
  kinRmLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinRmLeg.AddJointMDH( 0.0 , 0.0, -PI/2, 0.0 );
  kinRmLeg.AddJointMDH( 0.0 , lm ,  0.0 , 0.0 );
  kinRmLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinRmLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinRmLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );

  kinRfLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinRfLeg.AddJointMDH( 0.0 , 0.0, -0.4437, 0.0 );
  kinRfLeg.AddJointMDH( 0.0 , ll , -0.3416, 0.0 );
  kinRfLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinRfLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinRfLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );

  kinLfLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinLfLeg.AddJointMDH( 0.0 , 0.0, 0.4437 , 0.0 );
  kinLfLeg.AddJointMDH( 0.0 , ll , 0.3416 , 0.0 );
  kinLfLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinLfLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinLfLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );

  kinLmLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinLmLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinLmLeg.AddJointMDH( 0.0 , lm ,  0.0 , 0.0 );
  kinLmLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinLmLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinLmLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );

  kinLbLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinLbLeg.AddJointMDH( 0.0 , 0.0, 2.6978 , 0.0 );
  kinLbLeg.AddJointMDH( 0.0 , ll ,-0.3416 , 0.0 );
  kinLbLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinLbLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinLbLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );

  kinRbLeg.AddJointMDH( 0.0 , 0.0,  PI/2, 0.0 );
  kinRbLeg.AddJointMDH( 0.0 , 0.0, 3.5853 , 0.0 );
  kinRbLeg.AddJointMDH( 0.0 , ll , 0.3416 , 0.0 );
  kinRbLeg.AddJointMDH( PI/2, l1 ,  0.0 , 0.0 );
  kinRbLeg.AddJointMDH( 0.0 , l2 , -PI/2, 0.0 );
  kinRbLeg.AddJointMDH( 0.0 , l3 ,  0.0 , 0.0 );
      
}

void Haoze_H1mini::printcaliData(){
  signed char rec_lin;
  Serial.println();
  Serial.println("The following is the Calibrate data :");
  Serial.println("+---+---+---+---+---+---+---+---+---+---+----+----+----+----+----+----+----+----+");
  Serial.println("| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 |");
  Serial.println("+---+---+---+---+---+---+---+---+---+---+----+----+----+----+----+----+----+----+");
  
  char buffer[200]; // Adjust the size based on your needs
  snprintf(buffer, sizeof(buffer), "|%-3d|%-3d|%-3d|%-3d|%-3d|%-3d|%-3d|%-3d|%-3d|%-3d| %-3d| %-3d| %-3d| %-3d| %-3d| %-3d| %-3d| %-3d|", rec[0] , rec[1], rec[2], rec[3], rec[4], rec[5], rec[6], rec[7], rec[8], rec[9], rec[10], rec[11], rec[12], rec[13], rec[14], rec[15], rec[16], rec[17]);
  Serial.println(buffer);
  Serial.println("+---+---+---+---+---+---+---+---+---+---+----+----+----+----+----+----+----+----+");
  
  Serial.println();
}

void Haoze_H1mini::connect(){
    

}
//舵机驱动函数,参数：控制板舵机接口号，角度值
void Haoze_H1mini::servowrite(unsigned int id,float angle){
  if((id>=0)&&(id<16))
  pwm.setPWM(id, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
  else if((id>=16)&&(id<32))
  pwm1.setPWM(id-16, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
}
//关节驱动函数,参数：关节号，角度值
void Haoze_H1mini::jointwrite(unsigned int id,float angle){
  if((Jointservo[id]>=0)&&(Jointservo[id]<16))
  pwm.setPWM(Jointservo[id], 0, map(angle*direct[id]+rec[id],-90,90,SERVOMIN,SERVOMAX));
  else if((Jointservo[id]>=16)&&(Jointservo[id]<32))
  pwm1.setPWM(Jointservo[id]-16, 0, map(angle*direct[id]+rec[id],-90,90,SERVOMIN,SERVOMAX));
}

//关节执行函数，通过执行该函数，将会驱动18个关节舵机运动至s[18]数组传递的角度位置
void Haoze_H1mini::framewrite(float angle[18]){
  for(int i=0;i<18;i++)
  {
    jointwrite(i,angle[i]);
    Joint_P[i] = angle[i];
  }
}

//关节执行函数，通过执行该函数，将会驱动18个关节舵机运动至s[18]数组传递的角度位置
void Haoze_H1mini::frame2frame(float angle_p[18],float angle_r[18]){
  for(int k=0;k<20;k++)
  {
          for(int i=0;i<18;i++)
          {
            Servo_p[i] = angle_p[i]+(angle_r[i]-angle_p[i])*k/20.0;
          }
          framewrite(Servo_p);
          delay(speed);  
  }
  for(int i=0;i<18;i++)
  Servo_p[i]=Servo_r[i];//更新当前关节变量
}

void Haoze_H1mini::forward(unsigned int step){
  if(stepheight == 2)
  {
    for(int i=0;i<step;i++)
    {
      for(int k=0;k<40;k++)
      {
          framewrite(forward2[k]);
          delay(speed);
      }
    }
  }
  else if(stepheight == 3)
  {
    for(int i=0;i<step;i++)
    {
      for(int k=0;k<40;k++)
      {
          framewrite(forward3[k]);
          delay(speed);
      }
    }
  }
  else
  {
    for(int i=0;i<step;i++)
    {
      for(int k=0;k<40;k++)
      {
          framewrite(forward1[k]);
          delay(speed);
      }
    }
  }
}

void Haoze_H1mini::backward(unsigned int step){
  if(stepheight == 2)
  {
    for(int i=0;i<step;i++)
    {
      for(int k=39;k>=0;k--)
      {
          framewrite(forward2[k]);
          delay(speed);
      }
    }
  }
  else if(stepheight == 3)
  {
    for(int i=0;i<step;i++)
    {
      for(int k=39;k>=0;k--)
      {
          framewrite(forward3[k]);
          delay(speed);
      }
    }
  }
  else
  {
    for(int i=0;i<step;i++)
    {
      for(int k=39;k>=0;k--)
      {
          framewrite(forward1[k]);
          delay(speed);
      }
    }
  }

}

void Haoze_H1mini::turnleft(unsigned int step){
  for(int k = 0;k<step;k++)
  {
    if(stepheight == 1)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward1[20+j][i];

            Servo_p[15] = forward1[20+j][15];
            Servo_p[16] = forward1[20+j][16];
            Servo_p[17] = forward1[20+j][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward1[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward1[j-20][i];

            Servo_p[15] = forward1[j-20][15];
            Servo_p[16] = forward1[j-20][16];
            Servo_p[17] = forward1[j-20][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward1[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          }    
        }
        else if(stepheight == 2)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward2[20+j][i];

            Servo_p[15] = forward2[20+j][15];
            Servo_p[16] = forward2[20+j][16];
            Servo_p[17] = forward2[20+j][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward2[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward2[j-20][i];

            Servo_p[15] = forward2[j-20][15];
            Servo_p[16] = forward2[j-20][16];
            Servo_p[17] = forward2[j-20][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward2[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          }    
        }
        else if(stepheight == 3)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward3[20+j][i];

            Servo_p[15] = forward3[20+j][15];
            Servo_p[16] = forward3[20+j][16];
            Servo_p[17] = forward3[20+j][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward3[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward3[j-20][i];

            Servo_p[15] = forward3[j-20][15];
            Servo_p[16] = forward3[j-20][16];
            Servo_p[17] = forward3[j-20][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward3[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          }    
        }
  }


}

void Haoze_H1mini::turnright(unsigned int step){
  for(int k = 0;k<step;k++)
  {
        if(stepheight == 1)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward1[39-j][i];

            Servo_p[15] = forward1[39-j][15];
            Servo_p[16] = forward1[39-j][16];
            Servo_p[17] = forward1[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forward1[20+j][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward1[39-j][i];

            Servo_p[15] = forward1[39-j][15];
            Servo_p[16] = forward1[39-j][16];
            Servo_p[17] = forward1[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forward1[j-20][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          }
        }
        else if(stepheight == 2)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward2[39-j][i];

            Servo_p[15] = forward2[39-j][15];
            Servo_p[16] = forward2[39-j][16];
            Servo_p[17] = forward2[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forward2[20+j][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward2[39-j][i];

            Servo_p[15] = forward2[39-j][15];
            Servo_p[16] = forward2[39-j][16];
            Servo_p[17] = forward2[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forward2[j-20][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          }
        }
        else if(stepheight == 3)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward3[39-j][i];

            Servo_p[15] = forward3[39-j][15];
            Servo_p[16] = forward3[39-j][16];
            Servo_p[17] = forward3[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forward3[20+j][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forward3[39-j][i];

            Servo_p[15] = forward3[39-j][15];
            Servo_p[16] = forward3[39-j][16];
            Servo_p[17] = forward3[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forward3[j-20][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();                    //喂狗防止复位
            delay(speed);
          }
        }
  }
}

void Haoze_H1mini::leftward(unsigned int step){
  for(int i=0;i<step;i++)
  {
      for(int j=0;j<60;j++)
      {
          framewrite(forwardFH[j]);
          delay(speed);
      }
  }
}

void Haoze_H1mini::rightward(unsigned int step){
  for(int i=0;i<step;i++)
  {
      for(int j=0;j<60;j++)
      {
          framewrite(forwardFH[59-j]);
          delay(speed);
      }
  }
}

//初始化关节变量
void Haoze_H1mini::SetStartJointstate()
{
  kinRmLeg.JointState[3] = PI/6;
  kinRmLeg.JointState[4] = -PI/6;
  
  kinRfLeg.JointState[3] = PI/6;
  kinRfLeg.JointState[4] = -PI/6;
  
  kinLfLeg.JointState[3] = PI/6;
  kinLfLeg.JointState[4] = -PI/6;
  
  kinLmLeg.JointState[3] = PI/6;
  kinLmLeg.JointState[4] = -PI/6;
  
  kinLbLeg.JointState[3] = PI/6;
  kinLbLeg.JointState[4] = -PI/6;
  
  kinRbLeg.JointState[3] = PI/6;
  kinRbLeg.JointState[4] = -PI/6;
}

//正运动学然后获取Basefoot矩阵
void Haoze_H1mini::ForwardKinematics_GetBasefootM()
{
    SetStartJointstate();
    //正运动学
    kinRmLeg.forwardMDH();
    kinRfLeg.forwardMDH();
    kinLfLeg.forwardMDH();
    kinLmLeg.forwardMDH();
    kinLbLeg.forwardMDH();
    kinRbLeg.forwardMDH();
    //得到每条腿foot到base的齐次矩阵
    Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[0]);//B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[1]);//B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[2]);//B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[3]);//B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[4]);//B=A;齐次矩阵赋值
    Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[5]);//B=A;齐次矩阵赋值  
}

//正向运动学并得到BaselegM初始矩阵和其逆矩阵BaselegMatrixStartInvert
//Baseleg表示leg到base的矩阵，Start表示alpha和theta都是0.0，Inver表示逆矩阵
void Haoze_H1mini::Forward_GetBaselegMsv()
{
      kinRmLeg.Mdh[0][0] = 0.0;
      kinRfLeg.Mdh[0][0] = 0.0;
      kinLfLeg.Mdh[0][0] = 0.0;
      kinLmLeg.Mdh[0][0] = 0.0;
      kinLbLeg.Mdh[0][0] = 0.0;
      kinRbLeg.Mdh[0][0] = 0.0;
      
      kinRmLeg.Mdh[1][0] = 0.0;
      kinRfLeg.Mdh[1][0] = 0.0;
      kinLfLeg.Mdh[1][0] = 0.0;
      kinLmLeg.Mdh[1][0] = 0.0;
      kinLbLeg.Mdh[1][0] = 0.0;
      kinRbLeg.Mdh[1][0] = 0.0;

      kinRmLeg.JointState[0] = 0.0;
      kinRfLeg.JointState[0] = 0.0;
      kinLfLeg.JointState[0] = 0.0;
      kinLmLeg.JointState[0] = 0.0;
      kinLbLeg.JointState[0] = 0.0;
      kinRbLeg.JointState[0] = 0.0;
    
      //正运动学
      kinRmLeg.forwardMDH();
      kinRfLeg.forwardMDH();
      kinLfLeg.forwardMDH();
      kinLmLeg.forwardMDH();
      kinLbLeg.forwardMDH();
      kinRbLeg.forwardMDH();
  
      //首先找到leg根部到baselink矩阵，赋值给Base_legM
      Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[0]);//B=A;齐次矩阵赋值
      Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[1]);//B=A;齐次矩阵赋值
      Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[2]);//B=A;齐次矩阵赋值
      Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[3]);//B=A;齐次矩阵赋值
      Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[4]);//B=A;齐次矩阵赋值
      Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[5]);//B=A;齐次矩阵赋值

      //然后求leg根部到baselink逆矩阵
      Matrix.Invert((mtx_type*)Base_legM[0], N);
      Matrix.Invert((mtx_type*)Base_legM[1], N);
      Matrix.Invert((mtx_type*)Base_legM[2], N);
      Matrix.Invert((mtx_type*)Base_legM[3], N);
      Matrix.Invert((mtx_type*)Base_legM[4], N);
      Matrix.Invert((mtx_type*)Base_legM[5], N);
}

void Haoze_H1mini::GetLegfootM()
{
  //然后逆矩阵右边乘以MatrixOb,得到的就是foot到leg根部的矩阵
  //C=A*B;
  //Multiply(mtx_type* A, mtx_type* B, int m, int p, int n, mtx_type* C)
  //Leg_footM = Base_legM-1 *(Base_legM * Leg_FootM) = Base_legM-1 * Base_FootM
  Matrix.Multiply((mtx_type*)Base_legM[0], (mtx_type*)Base_footM[0], N, N, N, (mtx_type*)Leg_footM[0]);
  Matrix.Multiply((mtx_type*)Base_legM[1], (mtx_type*)Base_footM[1], N, N, N, (mtx_type*)Leg_footM[1]);
  Matrix.Multiply((mtx_type*)Base_legM[2], (mtx_type*)Base_footM[2], N, N, N, (mtx_type*)Leg_footM[2]);
  Matrix.Multiply((mtx_type*)Base_legM[3], (mtx_type*)Base_footM[3], N, N, N, (mtx_type*)Leg_footM[3]);
  Matrix.Multiply((mtx_type*)Base_legM[4], (mtx_type*)Base_footM[4], N, N, N, (mtx_type*)Leg_footM[4]);
  Matrix.Multiply((mtx_type*)Base_legM[5], (mtx_type*)Base_footM[5], N, N, N, (mtx_type*)Leg_footM[5]);
}

//机械臂逆运动学，传入参数为zuobiao[3]，目标的xyz三个坐标，
//解算结果存储在theta_rh[0]，theta_rh[1]，theta_rh[2]这三个变量中,根据Oa4坐标逆解
void Haoze_H1mini::ik4(float zuobiao[3]){
  float x3,y3,z3;
  float x1,y1,z1,P1P3_2,P1P3,P321;
  float P210,P213,P310,P3P0_2,P3P0;//P3P0_2是P3P0的平方P310是角P310
  x3 = zuobiao[0];
  y3 = zuobiao[1];
  z3 = zuobiao[2];

  //先求θ1
  theta_rh[0] = atan(y3/x3);
  //再求θ3
  x1 = l1 * cos(theta_rh[0]);
  y1 = l1 * sin(theta_rh[0]);
  z1 = 0.0;
  P1P3_2 = (x3-x1)*(x3-x1)+(y3-y1)*(y3-y1)+(z3-z1)*(z3-z1);
  P1P3 = sqrt(P1P3_2);
  P321=acos((l2*l2+l3*l3-P1P3_2)/(2*l2*l3));
  theta_rh[2] = P321-PI/2;
  
  //最后再求θ2,这里一定记得用余弦定理
  P213=acos((l2*l2+P1P3_2-l3*l3)/(2*l2*P1P3));
  P3P0_2=x3*x3+y3*y3+z3*z3;
  P3P0 = sqrt(P3P0_2);
  P310 = acos((l1*l1+P1P3_2-P3P0_2)/(2*l1*P1P3));
  theta_rh[1] = P213 + P310 - PI;
  
//  Serial.print("theta_rh1:");
//  Serial.print(theta_rh[0]);
//  Serial.print("   ");
//  Serial.print("theta_rh2:");
//  Serial.print(theta_rh[1]);
//  Serial.print("   ");
//  Serial.print("theta_rh3:");
//  Serial.println(theta_rh[2]);  
  
}

//根据得到的Endpose_3这个列向量得知Legfoot三维坐标，
//通过ik4算法计算关节变量，然后再赋值给H1mini关节变量的18个成员
void Haoze_H1mini::GetLegfootpose_IK2H1mini_Jointstate()
{
    //根据坐标逆解关节变量，OK了。
    //右中腿
    float zuobiao1[3];
    zuobiao1[0] = Endpose_3[0][0][0];
    zuobiao1[1] = Endpose_3[0][1][0];
    zuobiao1[2] = Endpose_3[0][2][0];
    ik4(zuobiao1);
    Servo_r[0] = theta_rh[0]*180.0/PI;
    Servo_r[1] = theta_rh[1]*180.0/PI;
    Servo_r[2] = theta_rh[2]*180.0/PI;
    //右前腿
    zuobiao1[0] = Endpose_3[1][0][0];
    zuobiao1[1] = Endpose_3[1][1][0];
    zuobiao1[2] = Endpose_3[1][2][0];
    ik4(zuobiao1);
    Servo_r[3] = theta_rh[0]*180.0/PI;
    Servo_r[4] = theta_rh[1]*180.0/PI;
    Servo_r[5] = theta_rh[2]*180.0/PI;
  
    //左前腿
    zuobiao1[0] = Endpose_3[2][0][0];
    zuobiao1[1] = Endpose_3[2][1][0];
    zuobiao1[2] = Endpose_3[2][2][0];
    ik4(zuobiao1);
    Servo_r[6] = theta_rh[0]*180.0/PI;
    Servo_r[7] = theta_rh[1]*180.0/PI;
    Servo_r[8] = theta_rh[2]*180.0/PI;
  
    //左中腿
    zuobiao1[0] = Endpose_3[3][0][0];
    zuobiao1[1] = Endpose_3[3][1][0];
    zuobiao1[2] = Endpose_3[3][2][0];
    ik4(zuobiao1);
    Servo_r[9] = theta_rh[0]*180.0/PI;
    Servo_r[10] = theta_rh[1]*180.0/PI;
    Servo_r[11] = theta_rh[2]*180.0/PI;
  
    //左后腿
    zuobiao1[0] = Endpose_3[4][0][0];
    zuobiao1[1] = Endpose_3[4][1][0];
    zuobiao1[2] = Endpose_3[4][2][0];
    ik4(zuobiao1);
    Servo_r[12] = theta_rh[0]*180.0/PI;
    Servo_r[13] = theta_rh[1]*180.0/PI;
    Servo_r[14] = theta_rh[2]*180.0/PI;
  
    //左后腿
    zuobiao1[0] = Endpose_3[5][0][0];
    zuobiao1[1] = Endpose_3[5][1][0];
    zuobiao1[2] = Endpose_3[5][2][0];
    ik4(zuobiao1);
    Servo_r[15] = theta_rh[0]*180.0/PI;
    Servo_r[16] = theta_rh[1]*180.0/PI;
    Servo_r[17] = theta_rh[2]*180.0/PI;
//    for(int i=0;i<18;i++)
//    {
//      Serial.print("S");
//      Serial.print(i);
//      Serial.print(":");
//      Serial.print(Haoze_h1mini.Servo_r[i]);
//      Serial.print("  ");
//    }
//    Serial.println();
}

void Haoze_H1mini::printLegFootpose()
{
  char buffer[50]; // Adjust the size based on your needs
  for(int i=0;i<6;i++)
  {
    Serial.print("Leg End_effectorPoint:");
    Serial.print("|");
    snprintf(buffer, sizeof(buffer), " x: %-9.2f y: %-9.2f z: %-9.2f ", Endpose_3[i][0][0], Endpose_3[i][1][0], Endpose_3[i][2][0]);
    Serial.print(buffer);
    Serial.println(); 
  }
}

//前进起步
void Haoze_H1mini::StartForward()
{
  
  ForwardKinematics_GetBasefootM();
  //    kinRmLeg.PrintJointstate();
//    printHaozeFootPoint();
//    printBaseFootpose();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
}

//前进收腿
void Haoze_H1mini::EndForward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
}

//机器人前进函数,参数ylength为前进距离毫米
void Haoze_H1mini::forward_y(float ylength)
{
  //首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //起步
  Serial.println("StartForwardy");
  StartForward();

  //循环走路
  //步子距离2*PI*stepx
  for(int k = 0;k<m;k++)
  {
    Serial.println("Forward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z + zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] - zcali;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] - zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z + zcali;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(forward_delay);
    }
  }
  //收腿
    EndForward();

    
  //最后一步
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartForward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z + zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] - zcali;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] - zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z + zcali;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(forward_delay);
    }
    Serial.println("EndForward");
    //收腿
    EndForward();
}

//后退起步
void Haoze_H1mini::StartBackward()
{
  
  ForwardKinematics_GetBasefootM();
  //    kinRmLeg.PrintJointstate();
//    printHaozeFootPoint();
//    printBaseFootpose();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
    
}

//后退收腿
void Haoze_H1mini::EndBackward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
}

//这个函数控制机器人后退多少距离
void Haoze_H1mini::backward_y(float ylength)
{
//首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //起步
  Serial.println("StartBackward");
  StartBackward();

  //循环走路
  //步子距离2*PI*stepx
  for(int k = 0;k<m;k++)
  {
    Serial.println("Backward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(backward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(backward_delay);
    }
  }
  //收腿
    EndBackward();

    
  //最后一步
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartBackward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3];
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3];
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3];
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(backward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3];
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3];
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3];
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(backward_delay);
    }
    Serial.println("EndBackward");
    //收腿
    EndBackward();
}

//右移起步
void Haoze_H1mini::StartRightward()
{
  
  ForwardKinematics_GetBasefootM();
  //    kinRmLeg.PrintJointstate();
//    printHaozeFootPoint();
//    printBaseFootpose();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
    
}

//右移收腿
void Haoze_H1mini::EndRightward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
}

void Haoze_H1mini::rightward_x(float ylength)
{
  //首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //起步
  Serial.println("StartRightwardx");
  StartRightward();

  //循环走路
  //步子距离2*PI*stepx
  for(int k = 0;k<m;k++)
  {
    Serial.println("Rightward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z + zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] - zcali;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] - zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z + zcali;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(forward_delay);
    }
  }
  //收腿
    EndRightward();

    
  //最后一步
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartRightward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z + zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] - zcali;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(forward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] - zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z + zcali;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(forward_delay);
    }
    Serial.println("EndRightward");
    //收腿
    EndRightward();
}

//左移起步
void Haoze_H1mini::StartLeftward()
{
  
  ForwardKinematics_GetBasefootM();
  //    kinRmLeg.PrintJointstate();
//    printHaozeFootPoint();
//    printBaseFootpose();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
}

//左移收腿
void Haoze_H1mini::EndLeftward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //左前腿
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //左后腿
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右前腿
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //左中腿
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //右后腿
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(20);
    }
}

void Haoze_H1mini::leftward_x(float ylength)
{
  //首先确定每一步走多少，接下来计算一共走几步，最后一步走多远
  ylength = ylength/1.71;//奇怪的参数，需要调整一下1.6552
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //起步
  Serial.println("StartBackward");
  StartLeftward();

  //循环走路
  //步子距离2*PI*stepx
  for(int k = 0;k<m;k++)
  {
    Serial.println("Backward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z + zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] - zcali;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
//      printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(backward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] - zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z + zcali;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(backward_delay);
    }
  }
  //收腿
    EndLeftward();

    
  //最后一步
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartLeftward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z + zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] - zcali;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(backward_delay);
    }
  
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] - zcali;
  
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z + zcali;
  
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] - zcali;
    
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z + zcali;
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
    
      GetLegfootM();  
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(backward_delay);
    }
    Serial.println("EndBackward");
    //收腿
    EndLeftward();
}

void Haoze_H1mini::start_turnleft()
{
    //准备姿态,0到10度。扭身子10到-10
    for(int i=0;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
  
}

void Haoze_H1mini::end_turnleft()
{
    //准备姿态,10到0度
    for(int i=10;i>=0;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
    framewrite(joint_s);
    delay(leftward_delay);
}

void Haoze_H1mini::turnleft_angle(float angleleft)
{
   angleleft = angleleft*1.2456;//不知名比例，要消去1.2456
  //计算循环旋转多少次
  //旋转起步
  angleleft = ((angleleft/180.0)*PI);
//  baseanglez = 400.0/angleleft;//每次40，循环10次，所以这里是400
  //这里计算循环多少次，每次旋转至少30度，弧度为zhuanwanjiaodu弧度
  int left_time = (int)(angleleft/(zhuanwanjiaodu));
  baseanglez = 40.0/zhuanwanjiaodu;
  start_turnleft();
  delay(500);
  //原地左转
  for(int m = 0;m<left_time;m++)
  {
    //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
    //扭身子-10到10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
  }
  delay(500);
  end_turnleft();
  delay(500);
  //最后一步旋转角度:
  //旋转收腿
  angleleft = angleleft - zhuanwanjiaodu * left_time;
  baseanglez = 40.0/angleleft;
  start_turnleft();
  delay(500);
  //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
    //扭身子10到-10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
    end_turnleft();
    delay(500);
}

//一次性左转多少度,一步完成,通常左转角度较小时应用,对称步态，需要起步和止步
//去掉多余的起步和止步，因为只是左转一步，为什么起步两次呢？当left_time为0时，不应该起步
void Haoze_H1mini::turnleft_angle1(float angleleft)
{
   angleleft = angleleft*1.2456;//不知名比例，要消去1.2456
  //计算循环旋转多少次
  //旋转起步
  angleleft = ((angleleft/180.0)*PI);
  //一步旋转角度，就是我们要转的角度
  baseanglez = 40.0/angleleft;
  start_turnleft();
  //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
    //扭身子10到-10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
    end_turnleft();
}

//一次性左转多少度,一步完成,通常左转角度较小时应用,非对称步态，不需要起步和止步
//去掉多余的起步和止步，因为只是左转一步，为什么起步两次呢？当left_time为0时，不应该起步
void Haoze_H1mini::turnleft_angle2(float angleleft)
{
   angleleft = angleleft*1.2456;//不知名比例，要消去1.2456
  //计算循环旋转多少次
  //旋转起步
  angleleft = ((angleleft/180.0)*PI);
  //一步旋转角度，就是我们要转的角度
  baseanglez = 40.0/angleleft;
  //扭身子10到-10
    for(int i=0;i>=-20;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(0到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;//(0到10)
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(-i)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
    //扭身子10到-10
    for(int i=-20;i<=0;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+20.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];

//      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
}

void Haoze_H1mini::start_turnright()
{
    //准备姿态,10到0度。扭身子10到-10
    for(int i=0;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
  
}

void Haoze_H1mini::end_turnright()
{
    //准备姿态,10到0度
    for(int i=-10;i<=0;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(leftward_delay);
    }
    framewrite(joint_s);
    delay(leftward_delay);
}

void Haoze_H1mini::turnright_angle(float angleright)
{
  angleright = angleright*1.2456;//不知名比例，要消去
  //计算循环旋转多少次
  //旋转起步
  angleright = ((angleright/180.0)*PI);
//  baseanglez = 400.0/angleright;//每次40，循环10次，所以这里是400
  //这里计算循环多少次，每次旋转至少30度，弧度为zhuanwanjiaodu弧度
  int right_time = (int)(angleright/(zhuanwanjiaodu));
  baseanglez = 40.0/zhuanwanjiaodu;
  start_turnright();
  delay(500);
  //原地右转
  for(int m = 0;m<right_time;m++)
  {
    //扭身子-10到10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(rightward_delay);
    }
    //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(rightward_delay);
    }
  }
  delay(500);
  end_turnright();
  delay(500);
  
  //最后一步旋转角度:
  //旋转收腿
  angleright = angleright - zhuanwanjiaodu * right_time;
  baseanglez = 40.0/angleright;
  start_turnright();
  delay(500);
  //最后一步
  //扭身子-10到10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(rightward_delay);
    }
    //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(rightward_delay);
    }
    delay(500);
    end_turnright();
    delay(500);
}

//一次性左转多少度,一步完成,通常左转角度较小时应用
//去掉多余的起步和止步，因为只是左转一步，为什么起步两次呢？当right_time为0时，不应该起步
void Haoze_H1mini::turnright_angle1(float angleright)
{
  angleright = angleright*1.2456;//不知名比例，要消去
  //计算循环旋转多少次
  //旋转起步
  angleright = ((angleright/180.0)*PI);
  //最后一步旋转角度:
  baseanglez = 40.0/angleright;
  start_turnright();
  //最后一步
  //扭身子-10到10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(rightward_delay);
    }
    //扭身子10到-10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(rightward_delay);
    }
    end_turnright();
}

//一次性右转多少度,一步完成,通常右转角度较小时应用,非对称步态，不需要起步和止步
//去掉多余的起步和止步，因为只是右转一步，为什么起步两次呢？当right_time为0时，不应该起步
void Haoze_H1mini::turnright_angle2(float angleright)
{
  angleright = angleright*1.2456;//不知名比例，要消去
  //计算循环旋转多少次
  //旋转起步
  angleright = ((angleright/180.0)*PI);
  //最后一步旋转角度:
  baseanglez = 40.0/angleright;
  //最后一步
  //扭身子-10到10
    for(int i=0;i<=20;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10到-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(rightward_delay);
    }
    //扭身子10到-10
    for(int i=20;i>=0;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
      float t = 2.0*PI*(20-i)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      //右中腿
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
      //foot到leg根部的齐次矩阵里面最后一列就是坐标了
      for(int i=0;i<6;i++)
      {
        Endpose_3[i][0][0] = Leg_footM[i][0][3];//x
        Endpose_3[i][1][0] = Leg_footM[i][1][3];//y
        Endpose_3[i][2][0] = Leg_footM[i][2][3];//z
        Endpose_3[i][3][0] = 1.0;
      }
  //    printLegFootpose();
      GetLegfootpose_IK2H1mini_Jointstate();
      framewrite(Servo_r);
      delay(rightward_delay);
    }
}


void Haoze_Q1mini::init(){
  Serial.begin(115200);
  Serial.println();
  Serial.println("Haoze_Q1mini QurapedRobot program!");

  pwm = Adafruit_PWMServoDriver();               //驱动1~16或(0~15)号舵机
  pwm1 = Adafruit_PWMServoDriver(0x41);          //驱动17~32或(16~31)号舵机

  Wire.begin();//开启IIC通信
  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  signed char rec_lin;
  //存储下来0度的偏移量
  EEPROM.begin(4095);//初始化eeprom大小
  //打印出来eeprom区域的所有数据
  for(int i=0;i<12;i++)
  {
    rec_lin = EEPROM.read(100+i);
    rec[i] = rec_lin;
    Serial.print(rec[i]);Serial.print(",");
    delay(1);
  }
}

void Haoze_Q1mini::servowrite(unsigned int id,float angle){
  if((id>=0)&&(id<16))
  pwm.setPWM(id, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
  else if((id>=16)&&(id<32))
  pwm1.setPWM(id-16, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
}

void Haoze_Q1mini::Print_EEPROM(){
  signed char rec_lin;
  for(int i=0;i<12;i++)
  {
    rec_lin = EEPROM.read(100+i);
    Serial.print(rec_lin);Serial.print(",");
    delay(1);
  }
  Serial.println();
}


void Haoze_Armini4::init(){
  Serial.begin(115200);
  Serial.println();
  Serial.println("Haoze_Armini4 Robot program!");

  pwm = Adafruit_PWMServoDriver();               //驱动1~16或(0~15)号舵机
  pwm1 = Adafruit_PWMServoDriver(0x41);          //驱动17~32或(16~31)号舵机

  Wire.begin();//开启IIC通信
  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  signed char rec_lin;
  //存储下来0度的偏移量
  EEPROM.begin(4095);//初始化eeprom大小
  //判断eeprom是否经过初始化
  rec_lin = EEPROM.read(Armini4EF);//最后面的这一个代表初始化标志,127为已初始化，否则为未初始化
  delay(1);
  if(rec_lin != 127)
  {
    Serial.println("First startup HaozeArmini4 Program!");
    Serial.println("Write EEPROM...");
    for(int i=0;i<4;i++)
    {
      EEPROM.write(Armini4ES+i,0);
      EEPROM.commit();  
      delay(1);    
    }
    EEPROM.write(Armini4EF,127);
    EEPROM.commit();  
    Serial.println("Eeprom write done!");
  }
  else
  {
    //打印出来eeprom区域的所有数据
    for(int i=0;i<4;i++)
    {
      rec_lin = EEPROM.read(Armini4ES+i);
      rec[i] = rec_lin;
      Serial.print(rec[i]);Serial.print(",");
      delay(1);
    }
    Serial.println();
    Serial.println("Eeprom Read done!");
  }

}

//贝塞尔曲线计算函数
float Haoze_Armini4::bezierBlend(float t, float p0, float p1, float p2, float p3) {
  return (1 - t) * (1 - t) * (1 - t) * p0 + 3 * (1 - t) * (1 - t) * t * p1 + 3 * (1 - t) * t * t * p2 + t * t * t * p3;
}

float Haoze_Armini4::smoothSplineMove(float startAngle, float endAngle, int steps, int i) {
    float t = float(i) / float(steps);
    float angle = bezierBlend(t, startAngle, startAngle + 10, endAngle - 10, endAngle);
    return angle;
}

//舵机驱动函数,参数：控制板舵机接口号，角度值
void Haoze_Armini4::servowrite(unsigned int id,float angle){
  if((id>=0)&&(id<16))
  pwm.setPWM(id, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
  else if((id>=16)&&(id<32))
  pwm1.setPWM(id-16, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
}

//关节驱动函数,参数：关节号，角度值
void Haoze_Armini4::jointwrite(unsigned int id,float angle){
  if((Jointservo[id]>=0)&&(Jointservo[id]<16))
  pwm.setPWM(Jointservo[id], 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
  else if((Jointservo[id]>=16)&&(Jointservo[id]<32))
  pwm1.setPWM(Jointservo[id]-16, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
}

//关节执行函数，通过执行该函数，将会驱动18个关节舵机运动至s[18]数组传递的角度位置
void Haoze_Armini4::framewrite(float angle[4]){
  for(int i=0;i<4;i++)
  jointwrite(i,angle[i]);
}

//关节执行函数，通过执行该函数，将会驱动18个关节舵机运动至s[18]数组传递的角度位置
void Haoze_Armini4::flame2frame(float angle_p[4],float angle_r[4]){
  for(int k=0;k<40;k++)
  {
          for(int i=0;i<4;i++)
          {
            // Servo_p[i] = angle_p[i]+(angle_r[i]-angle_p[i])*k/20.0;//这是匀速
            Servo_p[i] = smoothSplineMove(angle_p[i], angle_r[i], 40, k);
          }

          framewrite(Servo_p);
          delay(speed);  
  }
  for(int i=0;i<4;i++)
  Servo_p[i]=Servo_r[i];//更新当前关节变量
}


void Haoze_Spider12::init(){
  Serial.begin(115200);
  Serial.println();
  Serial.println("Haoze_Spider12 QurapedRobot program!");

  pwm = Adafruit_PWMServoDriver();               //驱动1~16或(0~15)号舵机
  pwm1 = Adafruit_PWMServoDriver(0x41);          //驱动17~32或(16~31)号舵机

  Wire.begin();//开启IIC通信
  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  signed char rec_lin;
  //存储下来0度的偏移量
  EEPROM.begin(4095);//初始化eeprom大小
  //打印出来eeprom区域的所有数据
  for(int i=0;i<12;i++)
  {
    rec_lin = EEPROM.read(200+i);
    rec[i] = rec_lin;
    Serial.print(rec[i]);Serial.print(",");
    delay(1);
  }
}

void Haoze_Spider12::servowrite(unsigned int id,float angle){
  if((id>=0)&&(id<16))
  pwm.setPWM(id, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
  else if((id>=16)&&(id<32))
  pwm1.setPWM(id-16, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
}

void Haoze_Spider12::Print_EEPROM(){
  signed char rec_lin;
  for(int i=0;i<12;i++)
  {
    rec_lin = EEPROM.read(100+i);
    Serial.print(rec_lin);Serial.print(",");
    delay(1);
  }
  Serial.println();
}
