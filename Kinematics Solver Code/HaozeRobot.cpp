#include "Arduino.h"
#include "HaozeRobot.h"
#include "Stepdata.h"
#include <EEPROM.h>  

#define H1miniES 0           //Six-legged robot servo calibration
#define H1miniEF 4094        //Calibration Marker for Hexapod Robot
#define Q1miniES 100         //Servo Calibration for Quadrupedal Robots
#define Q1miniEF 4094         //Calibration Marker for Quadruped Robot
#define Armini4ES 200        //Robotic Arm Servo Calibration
#define Armini4EF 399        //Robotic Arm Calibration Marker



void Haoze_H1mini::init(){
  Serial.begin(115200);
  Serial.println();
  Serial.println("Haoze_H1mini HexapodRobot program!");

  pwm = Adafruit_PWMServoDriver();               //Drive servos 1 to 16 or (0 to 15)
  pwm1 = Adafruit_PWMServoDriver(0x41);          //Drive servos 17 to 32 or (16 to 31)

  Wire.begin();
  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  signed char rec_lin;
  
  EEPROM.begin(4095);

  
  rec_lin = EEPROM.read(H1miniEF);
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


  //Initialise the six-legged MDH parameters
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
//Servo Drive Function Parameters: Control board servo interface number, angle value
void Haoze_H1mini::servowrite(unsigned int id,float angle){
  if((id>=0)&&(id<16))
  pwm.setPWM(id, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
  else if((id>=16)&&(id<32))
  pwm1.setPWM(id-16, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
}
//Joint Drive Function, Parameters: Joint Number, Angle Value
void Haoze_H1mini::jointwrite(unsigned int id,float angle){
  if((Jointservo[id]>=0)&&(Jointservo[id]<16))
  pwm.setPWM(Jointservo[id], 0, map(angle*direct[id]+rec[id],-90,90,SERVOMIN,SERVOMAX));
  else if((Jointservo[id]>=16)&&(Jointservo[id]<32))
  pwm1.setPWM(Jointservo[id]-16, 0, map(angle*direct[id]+rec[id],-90,90,SERVOMIN,SERVOMAX));
}

//Joint execution function: executing this function will drive the 18 joint servos to move to the angular positions specified in the s[18] array.
void Haoze_H1mini::framewrite(float angle[18]){
  for(int i=0;i<18;i++)
  {
    jointwrite(i,angle[i]);
    Joint_P[i] = angle[i];
  }
}

//Joint execution function: executing this function will drive the 18 joint servos to move to the angular positions specified in the s[18] array.
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
  Servo_p[i]=Servo_r[i];//Update the current joint variable
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
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward1[20+j][i];

            Servo_p[15] = forward1[20+j][15];
            Servo_p[16] = forward1[20+j][16];
            Servo_p[17] = forward1[20+j][17];
            
            //Left legs       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward1[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();  
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward1[j-20][i];

            Servo_p[15] = forward1[j-20][15];
            Servo_p[16] = forward1[j-20][16];
            Servo_p[17] = forward1[j-20][17];
            
            //Left legs       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward1[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();  
            delay(speed);
          }    
        }
        else if(stepheight == 2)
        {
          for(int j=0;j<20;j++)
          {
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward2[20+j][i];

            Servo_p[15] = forward2[20+j][15];
            Servo_p[16] = forward2[20+j][16];
            Servo_p[17] = forward2[20+j][17];
            
            //Left legs       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward2[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();  
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward2[j-20][i];

            Servo_p[15] = forward2[j-20][15];
            Servo_p[16] = forward2[j-20][16];
            Servo_p[17] = forward2[j-20][17];
            
            //Left legs       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward2[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();  
            delay(speed);
          }    
        }
        else if(stepheight == 3)
        {
          for(int j=0;j<20;j++)
          {
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward3[20+j][i];

            Servo_p[15] = forward3[20+j][15];
            Servo_p[16] = forward3[20+j][16];
            Servo_p[17] = forward3[20+j][17];
            
            //Left legs       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward3[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();  
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward3[j-20][i];

            Servo_p[15] = forward3[j-20][15];
            Servo_p[16] = forward3[j-20][16];
            Servo_p[17] = forward3[j-20][17];
            
            //Left legs       
            for(int i=6;i<15;i++)
            Servo_p[i] = forward3[39-j][i];
                
            framewrite(Servo_p);
            
            ESP.wdtFeed();  
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
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward1[39-j][i];

            Servo_p[15] = forward1[39-j][15];
            Servo_p[16] = forward1[39-j][16];
            Servo_p[17] = forward1[39-j][17];

            //Left legs
            for(int i=6;i<15;i++)
            Servo_p[i] = forward1[20+j][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();  
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward1[39-j][i];

            Servo_p[15] = forward1[39-j][15];
            Servo_p[16] = forward1[39-j][16];
            Servo_p[17] = forward1[39-j][17];

            //Left legs
            for(int i=6;i<15;i++)
            Servo_p[i] = forward1[j-20][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();  
            delay(speed);
          }
        }
        else if(stepheight == 2)
        {
          for(int j=0;j<20;j++)
          {
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward2[39-j][i];

            Servo_p[15] = forward2[39-j][15];
            Servo_p[16] = forward2[39-j][16];
            Servo_p[17] = forward2[39-j][17];

            //Left legs
            for(int i=6;i<15;i++)
            Servo_p[i] = forward2[20+j][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();  
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward2[39-j][i];

            Servo_p[15] = forward2[39-j][15];
            Servo_p[16] = forward2[39-j][16];
            Servo_p[17] = forward2[39-j][17];

            //Left legs
            for(int i=6;i<15;i++)
            Servo_p[i] = forward2[j-20][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();  
            delay(speed);
          }
        }
        else if(stepheight == 3)
        {
          for(int j=0;j<20;j++)
          {
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward3[39-j][i];

            Servo_p[15] = forward3[39-j][15];
            Servo_p[16] = forward3[39-j][16];
            Servo_p[17] = forward3[39-j][17];

            //Left legs
            for(int i=6;i<15;i++)
            Servo_p[i] = forward3[20+j][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();  
            delay(speed);
          } 
          for(int j=20;j<40;j++)
          {
            //Right legs
            for(int i=0;i<6;i++)
            Servo_p[i] = forward3[39-j][i];

            Servo_p[15] = forward3[39-j][15];
            Servo_p[16] = forward3[39-j][16];
            Servo_p[17] = forward3[39-j][17];

            //Left legs
            for(int i=6;i<15;i++)
            Servo_p[i] = forward3[j-20][i];

            framewrite(Servo_p);
            
            ESP.wdtFeed();  
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

//Initialise joint variables
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

//Forward kinematics to obtain the Basefoot matrix
void Haoze_H1mini::ForwardKinematics_GetBasefootM()
{
    SetStartJointstate();
    //Forward kinematics
    kinRmLeg.forwardMDH();
    kinRfLeg.forwardMDH();
    kinLfLeg.forwardMDH();
    kinLmLeg.forwardMDH();
    kinLbLeg.forwardMDH();
    kinRbLeg.forwardMDH();
    //Obtain the homogeneous matrix from foot to base for each leg
    Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[0]);
    Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[1]);
    Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[2]);
    Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[3]);
    Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[4]);
    Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[5], N, N, (mtx_type*)Base_footM[5]);//B=A;Homogeneous matrix assignment  
}

// Forward kinematics to obtain the initial BaselegM matrix and its inverse BaselegMatrixStartInvert
// Baseleg denotes the leg-to-base matrix, Start indicates alpha and theta are both 0.0, Invert denotes the inverse matrix
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
    
      
      kinRmLeg.forwardMDH();
      kinRfLeg.forwardMDH();
      kinLfLeg.forwardMDH();
      kinLmLeg.forwardMDH();
      kinLbLeg.forwardMDH();
      kinRbLeg.forwardMDH();
  
      //First locate the leg root to the baselink matrix, assigning it to Base_legM
      Matrix.Copy((mtx_type*)kinRmLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[0]);
      Matrix.Copy((mtx_type*)kinRfLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[1]);
      Matrix.Copy((mtx_type*)kinLfLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[2]);
      Matrix.Copy((mtx_type*)kinLmLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[3]);
      Matrix.Copy((mtx_type*)kinLbLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[4]);
      Matrix.Copy((mtx_type*)kinRbLeg.MatrixOb[2], N, N, (mtx_type*)Base_legM[5]);

      //Then compute the inverse matrix from the root of leg to the baselink.
      Matrix.Invert((mtx_type*)Base_legM[0], N);
      Matrix.Invert((mtx_type*)Base_legM[1], N);
      Matrix.Invert((mtx_type*)Base_legM[2], N);
      Matrix.Invert((mtx_type*)Base_legM[3], N);
      Matrix.Invert((mtx_type*)Base_legM[4], N);
      Matrix.Invert((mtx_type*)Base_legM[5], N);
}

void Haoze_H1mini::GetLegfootM()
{
  //Then multiply the inverse matrix by MatrixOb on the right-hand side to obtain the matrix from the foot to the root of the leg.
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

//Robotic arm inverse kinematics, input parameters are zuobiao[3], the target's xyz coordinates,
//Solution results stored in the three variables theta_rh[0], theta_rh[1], theta_rh[2], based on Oa4 coordinate inverse solution
void Haoze_H1mini::ik4(float zuobiao[3]){
  float x3,y3,z3;
  float x1,y1,z1,P1P3_2,P1P3,P321;
  float P210,P213,P310,P3P0_2,P3P0;
  x3 = zuobiao[0];
  y3 = zuobiao[1];
  z3 = zuobiao[2];

  
  theta_rh[0] = atan(y3/x3);
 
  x1 = l1 * cos(theta_rh[0]);
  y1 = l1 * sin(theta_rh[0]);
  z1 = 0.0;
  P1P3_2 = (x3-x1)*(x3-x1)+(y3-y1)*(y3-y1)+(z3-z1)*(z3-z1);
  P1P3 = sqrt(P1P3_2);
  P321=acos((l2*l2+l3*l3-P1P3_2)/(2*l2*l3));
  theta_rh[2] = P321-PI/2;
  
 
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

// The three-dimensional coordinates of Legfoot are determined from the column vector Endpose_3 obtained,
// joint variables are calculated via the IK4 algorithm, then assigned to the 18 members of the H1mini joint variables.
void Haoze_H1mini::GetLegfootpose_IK2H1mini_Jointstate()
{
    
    // Right middle leg
    float zuobiao1[3];
    zuobiao1[0] = Endpose_3[0][0][0];
    zuobiao1[1] = Endpose_3[0][1][0];
    zuobiao1[2] = Endpose_3[0][2][0];
    ik4(zuobiao1);
    Servo_r[0] = theta_rh[0]*180.0/PI;
    Servo_r[1] = theta_rh[1]*180.0/PI;
    Servo_r[2] = theta_rh[2]*180.0/PI;
    //Right front leg
    zuobiao1[0] = Endpose_3[1][0][0];
    zuobiao1[1] = Endpose_3[1][1][0];
    zuobiao1[2] = Endpose_3[1][2][0];
    ik4(zuobiao1);
    Servo_r[3] = theta_rh[0]*180.0/PI;
    Servo_r[4] = theta_rh[1]*180.0/PI;
    Servo_r[5] = theta_rh[2]*180.0/PI;
  
    //Left front leg
    zuobiao1[0] = Endpose_3[2][0][0];
    zuobiao1[1] = Endpose_3[2][1][0];
    zuobiao1[2] = Endpose_3[2][2][0];
    ik4(zuobiao1);
    Servo_r[6] = theta_rh[0]*180.0/PI;
    Servo_r[7] = theta_rh[1]*180.0/PI;
    Servo_r[8] = theta_rh[2]*180.0/PI;
  
    //Left middle leg
    zuobiao1[0] = Endpose_3[3][0][0];
    zuobiao1[1] = Endpose_3[3][1][0];
    zuobiao1[2] = Endpose_3[3][2][0];
    ik4(zuobiao1);
    Servo_r[9] = theta_rh[0]*180.0/PI;
    Servo_r[10] = theta_rh[1]*180.0/PI;
    Servo_r[11] = theta_rh[2]*180.0/PI;
  
    //left hind leg
    zuobiao1[0] = Endpose_3[4][0][0];
    zuobiao1[1] = Endpose_3[4][1][0];
    zuobiao1[2] = Endpose_3[4][2][0];
    ik4(zuobiao1);
    Servo_r[12] = theta_rh[0]*180.0/PI;
    Servo_r[13] = theta_rh[1]*180.0/PI;
    Servo_r[14] = theta_rh[2]*180.0/PI;
  
    //left hind leg
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

//Forward and start
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
      // Right middle leg
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //Left front leg
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //left hind leg
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
      //The last column of the homogeneous matrix at the root of the foot to leg represents the coordinates.
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
      // Right middle leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      

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

//Step forward and draw legs back
void Haoze_H1mini::EndForward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //Right front leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //Left front leg
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //left hind leg
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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
      //Right front leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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

//Robot forward function, parameter ylength denotes forward distance in millimetres
void Haoze_H1mini::forward_y(float ylength)
{
  ylength = ylength/1.71;
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  
  Serial.println("StartForwardy");
  StartForward();

  
  for(int k = 0;k<m;k++)
  {
    Serial.println("Forward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
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
      // Right middle leg
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
    EndForward();

    
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartForward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
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
      // Right middle leg
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
    
    EndForward();
}

//Reverse start
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
      // Right middle leg
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //Left front leg
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //left hind leg
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] + x/2;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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
      // Right middle leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - x/2;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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

//Retreat and draw in your legs
void Haoze_H1mini::EndBackward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3];
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //Right front leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //Left front leg
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3];
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //left hind leg
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3];
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3] - x/2 + stepx*PI;
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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
      //Right front leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3];
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3];
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3];
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3] + x/2 - stepx*PI;
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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

//This function controls how far the robot moves backwards.
void Haoze_H1mini::backward_y(float ylength)
{
//First determine how far each step covers, then calculate the total number of steps, and finally determine the distance covered by the last step.
  ylength = ylength/1.71;
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  
  Serial.println("StartBackward");
  StartBackward();


  for(int k = 0;k<m;k++)
  {
    Serial.println("Backward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
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
      // Right middle leg
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

    EndBackward();

    

  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartBackward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
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
      // Right middle leg
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
      
    EndBackward();
}

//Right-hand drive start
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
      // Right middle leg
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //Left front leg
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //left hind leg
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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
      // Right middle leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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

//Shift right and draw in the leg
void Haoze_H1mini::EndRightward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //Right front leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //Left front leg
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //left hind leg
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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
      //Right front leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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
  ylength = ylength/1.71;  
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //Getting started
  Serial.println("StartRightwardx");
  StartRightward();

 
 
  for(int k = 0;k<m;k++)
  {
    Serial.println("Rightward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
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
      // Right middle leg
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
    
    EndRightward();

    
   //Final step
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartRightward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
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
      // Right middle leg
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
      
    EndRightward();
}

//left Getting started
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
      // Right middle leg
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //Left front leg
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //left hind leg
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] + x/2;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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
      // Right middle leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - x/2;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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

//left Draw legs in.
void Haoze_H1mini::EndLeftward()
{
  
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][0][3] = kinRmLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[0][1][3] = kinRmLeg.MatrixOb[5][1][3];
      Base_footM[0][2][3] = kinRmLeg.MatrixOb[5][2][3] + z;
      //Right front leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] ;
      //Left front leg
      Base_footM[2][0][3] = kinLfLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[2][1][3] = kinLfLeg.MatrixOb[5][1][3];
      Base_footM[2][2][3] = kinLfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] ;
      //left hind leg
      Base_footM[4][0][3] = kinLbLeg.MatrixOb[5][0][3] - x/2 + stepx*PI;
      Base_footM[4][1][3] = kinLbLeg.MatrixOb[5][1][3];
      Base_footM[4][2][3] = kinLbLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] ;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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
      //Right front leg
      Base_footM[1][0][3] = kinRfLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[1][1][3] = kinRfLeg.MatrixOb[5][1][3];
      Base_footM[1][2][3] = kinRfLeg.MatrixOb[5][2][3] + z;
      //Left middle leg
      Base_footM[3][0][3] = kinLmLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[3][1][3] = kinLmLeg.MatrixOb[5][1][3];
      Base_footM[3][2][3] = kinLmLeg.MatrixOb[5][2][3] + z;
      //right hind leg
      Base_footM[5][0][3] = kinRbLeg.MatrixOb[5][0][3] + x/2 - stepx*PI;
      Base_footM[5][1][3] = kinRbLeg.MatrixOb[5][1][3];
      Base_footM[5][2][3] = kinRbLeg.MatrixOb[5][2][3] + z;
  
  //    printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
      
 
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
  ylength = ylength/1.71;  
  stepx = 5.0;
  int m;
  m = (int)(ylength/(stepx*2*PI));
  //Getting started
  Serial.println("StartBackward");
  StartLeftward();

 
 
  for(int k = 0;k<m;k++)
  {
    Serial.println("Backward");
    ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
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
      // Right middle leg
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
    
    EndLeftward();

    
   //Final step
  stepx = (ylength - 2*PI*stepx*m)/(2*PI);
  StartLeftward();
  ForwardKinematics_GetBasefootM();
    for(int i=1;i<=20;i++)
    {
      float t = 2.0*PI*i/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
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
      // Right middle leg
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
      
    EndLeftward();
}

void Haoze_H1mini::start_turnleft()
{
    //Preparation posture, 0 to 10 degrees. Twist body 10 to -10 degrees.
    for(int i=0;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10 to -10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
    //Preparation posture, 10 to 0 degrees
    for(int i=10;i>=0;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10to-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
   angleleft = angleleft*1.2456;//Unknown proportion, to be eliminated: 1.2456
   //Calculate how many times the loop rotates
 //Rotation Getting started
  angleleft = ((angleleft/180.0)*PI);
  // baseanglez = 400.0 / angleleft; // Increments by 40, cycles 10 times, hence 400
  // Calculates the number of cycles, with each rotation at least 30 degrees, in radians as zhuanwanjiaodu radians
  int left_time = (int)(angleleft/(zhuanwanjiaodu));
  baseanglez = 40.0/zhuanwanjiaodu;
  start_turnleft();
  delay(500);
  //Turn left on the spot
  for(int m = 0;m<left_time;m++)
  {
      //Twist body 10 to -10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10to-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
    //Twist your body -10 to 10
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
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
  //Final step rotation angle:
//Rotate and draw legs in.
  angleleft = angleleft - zhuanwanjiaodu * left_time;
  baseanglez = 40.0/angleleft;
  start_turnleft();
  delay(500);
  //Twist body 10 to -10
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10to-10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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

//Single left turn by a specified degree, completed in one step. Typically used for small left turns, symmetrical gait, requiring Getting started and stopping.
//Remove redundant Getting started and stopping, as this is merely a single left turn. When left_time is 0, Getting started should not occur.
void Haoze_H1mini::turnleft_angle1(float angleleft)
{
   angleleft = angleleft*1.2456; 
  //Calculate how many times the loop rotates
  //Spinning start
  angleleft = ((angleleft/180.0)*PI);
   //The rotation angle per step is the angle we wish to rotate by.
  baseanglez = 40.0/angleleft;
  start_turnleft();
   
    for(int i=10;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10 to -10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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


void Haoze_H1mini::turnleft_angle2(float angleleft)
{
   angleleft = angleleft*1.2456; 
  //Calculate how many times the loop rotates
  //Spinning start
  angleleft = ((angleleft/180.0)*PI);
   //The rotation angle per step is the angle we wish to rotate by.
  baseanglez = 40.0/angleleft;
   
    for(int i=0;i>=-20;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(0 to -10)
      kinRfLeg.JointState[0] = -i/baseanglez;//(0 to 10)
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(-i)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;

//      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];

//      printBaseFootpose();
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
    //Preparation posture, 10 to 0 degrees. Twist body 10 to -10 degrees.
    for(int i=0;i>=-10;i--)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10 to -10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
    //prepare
    for(int i=-10;i<=0;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10 to -10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
  angleright = angleright*1.2456;
  //Calculate how many times the loop rotates
  //Spinning start
  angleright = ((angleright/180.0)*PI);

  int right_time = (int)(angleright/(zhuanwanjiaodu));
  baseanglez = 40.0/zhuanwanjiaodu;
  start_turnright();
  delay(500);
  //Turn right on the spot
  for(int m = 0;m<right_time;m++)
  {
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10 to -10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
  
  //Final step rotation angle:
 //Rotate and draw legs in.
  angleright = angleright - zhuanwanjiaodu * right_time;
  baseanglez = 40.0/angleright;
  start_turnright();
  delay(500);
  //Final step
//Twist body -10 to 10
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10 to -10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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

void Haoze_H1mini::turnright_angle1(float angleright)
{
  angleright = angleright*1.2456;
  //Calculate how many times the loop rotates
  //Spinning start
  angleright = ((angleright/180.0)*PI);
   //Final step:
  baseanglez = 40.0/angleright;
  start_turnright();
   //Final step
    for(int i=-10;i<=10;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10 to -10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i+10.0)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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


void Haoze_H1mini::turnright_angle2(float angleright)
{
  angleright = angleright*1.2456;
  //Calculate how many times the loop rotates
  //Spinning start
  angleright = ((angleright/180.0)*PI);
   //Final step
  baseanglez = 40.0/angleright;
   //Final step
    for(int i=0;i<=20;i++)
    {
      kinRmLeg.JointState[0] = i/baseanglez;//(10 to -10)
      kinRfLeg.JointState[0] = -i/baseanglez;
      kinLfLeg.JointState[0] = i/baseanglez;
      kinLmLeg.JointState[0] = -i/baseanglez;
      kinLbLeg.JointState[0] = i/baseanglez;
      kinRbLeg.JointState[0] = -i/baseanglez;
  
      ForwardKinematics_GetBasefootM();
  
      float t = 2.0*PI*(i)/20.0;
      float x=stepx*(t-sin(t));
      float z=stepz*(1-cos(t));
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3];
      Base_footM[1][2][3] = Base_footM[1][2][3] + 5.0 + z;
      Base_footM[2][2][3] = Base_footM[2][2][3];
      Base_footM[3][2][3] = Base_footM[3][2][3] + 5.0 + z;
      Base_footM[4][2][3] = Base_footM[4][2][3];
      Base_footM[5][2][3] = Base_footM[5][2][3] + 5.0 + z;
      
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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
      // Right middle leg
      Base_footM[0][2][3] = Base_footM[0][2][3] + 5.0 + z;
      Base_footM[1][2][3] = Base_footM[1][2][3];
      Base_footM[2][2][3] = Base_footM[2][2][3] + 5.0 + z;
      Base_footM[3][2][3] = Base_footM[3][2][3];
      Base_footM[4][2][3] = Base_footM[4][2][3] + 5.0 + z;
      Base_footM[5][2][3] = Base_footM[5][2][3];
      Forward_GetBaselegMsv();
      GetLegfootM();
        
 
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

  pwm = Adafruit_PWMServoDriver();               //Drive servos 1 to 16 or (0 to 15)
  pwm1 = Adafruit_PWMServoDriver(0x41);          //Drive servos 17 to 32 or (16 to 31)

  Wire.begin();
  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  signed char rec_lin;
  EEPROM.begin(4095);
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

  pwm = Adafruit_PWMServoDriver();               //Drive servos 1 to 16 or (0 to 15)
  pwm1 = Adafruit_PWMServoDriver(0x41);          //Drive servos 17 to 32 or (16 to 31)

  Wire.begin();
  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  signed char rec_lin;
  //Store the offset at 0 degrees
  EEPROM.begin(4095); 
  //Determine whether the EEPROM has been initialised
  rec_lin = EEPROM.read(Armini4EF);//The final one represents the initialisation flag: 127 indicates initialised, otherwise it is not initialised.
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
    //Print out all data from the EEPROM region
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

//Bézier curve calculation function
float Haoze_Armini4::bezierBlend(float t, float p0, float p1, float p2, float p3) {
  return (1 - t) * (1 - t) * (1 - t) * p0 + 3 * (1 - t) * (1 - t) * t * p1 + 3 * (1 - t) * t * t * p2 + t * t * t * p3;
}

float Haoze_Armini4::smoothSplineMove(float startAngle, float endAngle, int steps, int i) {
    float t = float(i) / float(steps);
    float angle = bezierBlend(t, startAngle, startAngle + 10, endAngle - 10, endAngle);
    return angle;
}

//Servo Drive Function Parameters: Control board servo interface number, angle value
void Haoze_Armini4::servowrite(unsigned int id,float angle){
  if((id>=0)&&(id<16))
  pwm.setPWM(id, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
  else if((id>=16)&&(id<32))
  pwm1.setPWM(id-16, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
}

//关Joint-driven function, parameters: joint number, angle value
void Haoze_Armini4::jointwrite(unsigned int id,float angle){
  if((Jointservo[id]>=0)&&(Jointservo[id]<16))
  pwm.setPWM(Jointservo[id], 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
  else if((Jointservo[id]>=16)&&(Jointservo[id]<32))
  pwm1.setPWM(Jointservo[id]-16, 0, map((angle+rec[id])*direct[id],-90,90,SERVOMIN,SERVOMAX));
}

//Joint execution function: executing this function will drive the 18 joint servos to move to the angular positions specified in the s[18] array.
void Haoze_Armini4::framewrite(float angle[4]){
  for(int i=0;i<4;i++)
  jointwrite(i,angle[i]);
}

void Haoze_Armini4::flame2frame(float angle_p[4],float angle_r[4]){
  for(int k=0;k<40;k++)
  {
          for(int i=0;i<4;i++)
          {
            // Servo_p[i] = angle_p[i]+(angle_r[i]-angle_p[i])*k/20.0;
            Servo_p[i] = smoothSplineMove(angle_p[i], angle_r[i], 40, k);
          }

          framewrite(Servo_p);
          delay(speed);  
  }
  for(int i=0;i<4;i++)
  Servo_p[i]=Servo_r[i];//Update the current joint variable
}


void Haoze_Spider12::init(){
  Serial.begin(115200);
  Serial.println();
  Serial.println("Haoze_Spider12 QurapedRobot program!");

  pwm = Adafruit_PWMServoDriver();               //Drive servos 1 to 16 or (0 to 15)
  pwm1 = Adafruit_PWMServoDriver(0x41);          //Drive servos 17 to 32 or (16 to 31)

  Wire.begin();
  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  signed char rec_lin;
  //Store the offset at 0 degrees
  EEPROM.begin(4095); 
  //Print out all data from the EEPROM region
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
