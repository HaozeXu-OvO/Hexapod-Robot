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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
            
            // ESP.wdtFeed();                    //喂狗防止复位
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
  pwm.setPWM(id, 0, map(angle*direct[id]+rec[id],-90,90,SERVOMIN,SERVOMAX));
  else if((id>=16)&&(id<32))
  pwm1.setPWM(id-16, 0, map(angle*direct[id]+rec[id],-90,90,SERVOMIN,SERVOMAX));
}

//关节驱动函数,参数：关节号，角度值
void Haoze_Armini4::jointwrite(unsigned int id,float angle){
  if((Jointservo[id]>=0)&&(Jointservo[id]<16))
  pwm.setPWM(Jointservo[id], 0, map(angle*direct[id]+rec[id],-90,90,SERVOMIN,SERVOMAX));
  else if((Jointservo[id]>=16)&&(Jointservo[id]<32))
  pwm1.setPWM(Jointservo[id]-16, 0, map(angle*direct[id]+rec[id],-90,90,SERVOMIN,SERVOMAX));
}

//关节执行函数，通过执行该函数，将会驱动18个关节舵机运动至s[18]数组传递的角度位置
void Haoze_Armini4::framewrite(float angle[4]){
  for(int i=0;i<4;i++)
  jointwrite(i,angle[i]);


  // Serial.print("Jointstate:");
  // for(int i=0;i<4;i++)
  // {
  //   Serial.print("J");
  //   Serial.print(i);
  //   Serial.print(":");
  //   Serial.print(angle[i]);
  //   Serial.print("  ");
  // }
  // Serial.println("   ");
  
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
