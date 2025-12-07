/*******************************************************
   主板：Haoze_Servo8266
   功能：Haoze_H1mini六足机器人Arduino程序
   引脚：SDA:21   SCL:22
   对于ARDUINO UNO，SCL:A5，SDA:A4
   Designer: Allen
   E-mail:953598974@qq.com
   Date:2024-08-19
*******************************************************/
#include <HaozeRobot.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>  //引入库文件
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "StepData.h"
Haoze_H1mini haoze;
//#include "PS2.h"
//关节标定数据存储起始位置
#define Servo_Addr 0

#define del 100
#define deltr 3
#define led 2               //led为低电平时，灯亮；高电平时，灯灭

#define wifi
//#define ps2

/***************************************************************************************/
#ifdef ps2

#include <PS2X_lib.h>  //for v1.6

#define PS2_DAT        13  //13    
#define PS2_CMD        15  //15
#define PS2_SEL        14  //14
#define PS2_CLK        12  //12

//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

#endif

/***************************************************************************************/

#ifdef wifi

#define MAX_SRV_CLIENTS 3   //最大同时联接数，即你想要接入的设备数量，8266tcpserver只能接入五个

const char *ssid = "Haoze"; 
const char *password = "haozerobot"; 
WiFiServer server(8266);//你要的端口号，随意修改，范围0-65535
WiFiClient serverClients[MAX_SRV_CLIENTS];
#endif

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();               //驱动1~16或(0~15)号舵机
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);          //驱动17~32或(16~31)号舵机

#define servo180     //如果你的舵机是180度舵机，那么将这一行代码取消注释，并将下一行代码注释掉
//#define servo270   //如果你的舵机是270度舵机，那么将这一行代码取消注释，并将上一行代码注释掉

#ifdef servo180
//#define SERVOMIN  102               //0.5/20 * 4096 = 102
//#define SERVOMID  307               //1.5/20 * 4096 = 307
//#define SERVOMAX  512               //2.5/20 * 4096 = 512
//实际测试
#define SERVOMIN  102               
#define SERVOMID  327               
#define SERVOMAX  552
#endif

#ifdef servo270
//#define SERVOMIN  102               //0.5/20 * 4096 = 102
//#define SERVOMID  307               //1.5/20 * 4096 = 307
//#define SERVOMAX  512               //2.5/20 * 4096 = 512
//实际测试
#define SERVOMIN  177           
#define SERVOMID  327               
#define SERVOMAX  577
#endif

//pwm.setPWM(i, 0, pulselen);第一个参数是通道数;第二个是高电平起始点，也就是从0开始;第三个参数是高电平终止点。

//a:前进; b:后退; c:左转; d:右转; e:停止; f:向左横行 g:向右横行 h:步态切换 i:身高切换 j:原地蹲起 k:跳跳；
//last_cmd表示上一个指令，可以表示机器人状态。
char cmd = 'e',last_cmd = 'e';
//gait表示步态，1为波动步态，0为三角步态；body表示身高，0为最低，1为中间，2为最高;robotstatus为机器人状态，
int gait=0,body=0,robotstatus=0;


//转弯半径控制比例,前进后退速度比例
float zhuanwan_k = 1;
float qianhou_k = 1;

//
signed char rec_lin;
signed char rec[18] = {
  0,0,0,
  0,0,0,
  0,0,0,
  0,0,0,
  0,0,0,
  0,0,0
};
//机器人当前关节变量，舵机当前角度
float Servo_p[18] = {
  0.0 , 0.0 , 0.0 ,
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0
};

//机器人当前关节变量，舵机当前角度
float zero_p[18] = {
  0.0 , 0.0 , 0.0 ,
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0
};
//机器人目标关节变量，舵机目标角度
float Servo_r[18] = {
  0.0 , 0.0 , 0.0 ,
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0 , 
  0.0 , 0.0 , 0.0
};
//舵机方向反相参数
int direct[18] = {-1,1,1,
-1,1,1,
-1,1,1,
-1,1,1,
-1,1,1,
-1,1,1
};

int gait_zhen = 0;

void iic_device_test()//扫描iic芯片，如果开机闪烁一次，说明是0x40未扫描到；如果闪烁两次，则是0x41未扫描到。
{
  bool iic_flag[2];//定义一个iic标志数组用于表示iic扫描结果
  Wire.beginTransmission(0x40);
  if(Wire.endTransmission()!=0)//0是扫描到设备了，非0是未扫描到设备。
  while(1)
  {
    digitalWrite(led,0);
    delay(100);
    digitalWrite(led,1);
    delay(1000);
  }
  Wire.beginTransmission(0x41);
  if(Wire.endTransmission()!=0)//0是扫描到设备了，非0是未扫描到设备。
  while(1)
  {
    digitalWrite(led,0);
    delay(100);
    digitalWrite(led,1);
    delay(100);
    digitalWrite(led,0);
    delay(100);
    digitalWrite(led,1);
    delay(1000);
  }
};

void print_jointjz(){
  Serial.println();
  Serial.println("The following is the biaoding :");
  Serial.print(" ");
  for(int i=0;i<18;i++)
  {
    Serial.print(i);
    Serial.print("    ");
  }
  Serial.println();
  for(int i=0;i<18;i++)
  {
    rec_lin = EEPROM.read(i);
    Serial.print(rec_lin);
    Serial.print("  ");
  }
  Serial.println();
}

void blink()
{
    static long previousMillis = 0;
    static int currstate = 0;
 
    if (millis() - previousMillis > 200)  //200ms
    {
        previousMillis = millis();
        currstate = 1 - currstate;
        digitalWrite(led, currstate);
    }
}

void setup() {
  // put your setup code here, to run once:
    //定义各个关节舵机接口号
  haoze.Jointservo[0] = 0;haoze.Jointservo[1] = 1;haoze.Jointservo[2] = 2;
  haoze.Jointservo[3] = 18;haoze.Jointservo[4] = 4;haoze.Jointservo[5] = 5;
  haoze.Jointservo[6] = 6;haoze.Jointservo[7] = 19;haoze.Jointservo[8] = 8;
  haoze.Jointservo[9] = 9;haoze.Jointservo[10] = 10;haoze.Jointservo[11] = 20;
  haoze.Jointservo[12] = 12;haoze.Jointservo[13] = 13;haoze.Jointservo[14] = 14;
  haoze.Jointservo[15] = 15;haoze.Jointservo[16] = 16;haoze.Jointservo[17] = 17;

  haoze.Jointservo[3] = 18;haoze.Jointservo[7] = 19;haoze.Jointservo[11] = 20;

  pinMode(led, OUTPUT);
  digitalWrite(led, 1);//led为低电平时，灯亮；高电平时，灯灭
  Serial.begin(115200);
  Serial.println();
  Serial.println("Haoze_H1mini HexapodRobot program!");

  Wire.begin();//开启IIC通信
  iic_device_test();
  Serial.println("iic!");
  
  pwm.begin();
  pwm1.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  Serial.println("PCA9685!");

  //存储下来0度的偏移量
  EEPROM.begin(4095);//初始化eeprom大小
  //判断eeprom是否经过初始化
  rec_lin = EEPROM.read(4094);//最后面的这一个代表初始化标志,127为已初始化，否则为未初始化
  delay(1);
  if(rec_lin != 127)
  {
    for(int i=0;i<18;i++)
    {
      EEPROM.write(Servo_Addr+i,0);
      EEPROM.commit();  
      delay(1);    
    }
    EEPROM.write(4094,127);
    EEPROM.commit();  
  }
  else;
  
  
  //打印出来eeprom区域的所有数据
  for(int i=0;i<18;i++)
  {
    rec_lin = EEPROM.read(i);
    rec[i] = rec_lin;
    Serial.print(rec[i]);Serial.print(",");
    delay(1);
  }  
  Serial.println();

  
  for(int i=0;i<18;i++)
  Servo_p[i] = float(rec[i]);
  ESP.wdtFeed();                    //喂狗防止复位
  delay(100);

  digitalWrite(led, 0);
  haoze.init();
  haoze.framewrite(zero_p);
  delay(100);
  haoze.framewrite(zero_p);
  delay(100);
  #ifdef wifi
  Serial.println();
  Serial.print("Connecting the hostspot");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
        delay(500);
        Serial.print(".");
  }
  Serial.println();
  Serial.print("The robot IP address is:");
  Serial.println(WiFi.localIP());
  server.begin();
  server.setNoDelay(true);  //加上后才正常些
  #endif


  #ifdef ps2
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
  if (pressures)
    Serial.println("true ");
  else
    Serial.println("false");
  Serial.print("rumble = ");
  if (rumble)
    Serial.println("true)");
  else
    Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
//  Serial.print(ps2x.Analog(1), HEX);
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
  case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }
   #endif

  delay(1000);

  for(int i=0;i<18;i++)
  Servo_p[i] = 0.0;

  //从标定姿态过渡到前进准备姿态
  haoze.framewrite(Servo_p);
  haoze.frame2frame(Servo_p,forwarda[0]);
  haoze.framewrite(forwarda[0]);  
}

void loop() {
   blink();
    #ifdef wifi
    uint8_t i;
    if (server.hasClient())
    {
        for (i = 0; i < MAX_SRV_CLIENTS; i++)
        {
            if (!serverClients[i] || !serverClients[i].connected())
            {
                if (serverClients[i]) serverClients[i].stop();//未联接,就释放
                serverClients[i] = server.available();//分配新的
                continue;
            }
 
        }
        WiFiClient serverClient = server.available();
        serverClient.stop();
    }
    for (i = 0; i < MAX_SRV_CLIENTS; i++)
    {
        if (serverClients[i] && serverClients[i].connected())
        {
            digitalWrite(led, 0);//有链接存在,就一直长亮
 
            if (serverClients[i].available())
            {
                while (serverClients[i].available()) 
                cmd = serverClients[i].read();
                delay(1);
                Serial.write(serverClients[i].read());
            }
        }
    }
    #endif


  #ifdef ps2
  if(error == 1) //skip loop if no controller found
    return; 
  
  if(type == 2){ //Guitar Hero Controller
    ps2x.read_gamepad();          //read controller 
   
    if(ps2x.ButtonPressed(GREEN_FRET))
      Serial.println("Green Fret Pressed");
    if(ps2x.ButtonPressed(RED_FRET))
      Serial.println("Red Fret Pressed");
    if(ps2x.ButtonPressed(YELLOW_FRET))
      Serial.println("Yellow Fret Pressed");
    if(ps2x.ButtonPressed(BLUE_FRET))
      Serial.println("Blue Fret Pressed");
    if(ps2x.ButtonPressed(ORANGE_FRET))
      Serial.println("Orange Fret Pressed"); 

    if(ps2x.ButtonPressed(STAR_POWER))
      Serial.println("Star Power Command");
    
    if(ps2x.Button(UP_STRUM))          //will be TRUE as long as button is pressed
      Serial.println("Up Strum");
    if(ps2x.Button(DOWN_STRUM))
      Serial.println("DOWN Strum");
 
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
    {
      Serial.println("Start is being held");
      cmd = 'e';
    }
      
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");
    
    if(ps2x.Button(ORANGE_FRET)) {     // print stick value IF TRUE
      Serial.print("Wammy Bar Position:");
      Serial.println(ps2x.Analog(WHAMMY_BAR), DEC); 
    } 
  }
  else { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
    {
      Serial.println("Start is being held");
      cmd = 'e';
    }
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");      

    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      cmd = 'a';
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
      cmd = 'd';
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
      cmd = 'c';
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
      cmd = 'b';
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }   

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if(ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if(ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if(ps2x.Button(PSB_TRIANGLE))
      {
        cmd = 'h';
        Serial.println("Triangle pressed"); 
      }
               
    }

    if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
    {
      cmd = 'g';
      Serial.println("Circle just pressed");
    }
      
    if(ps2x.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
      {
        cmd = 'i';
        Serial.println("X just changed");
      }
      
    if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
    {
      cmd = 'f';
      Serial.println("Square just released"); 
    }
          

    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC); 
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC); 
    }
  }     
  #endif
  
  while(Serial.available()>0)
  {
    cmd = Serial.read();
    if(cmd=='x')//校准舵机角度
    {
      while(Serial.available()>0)
      {
        int servo_id=Serial.parseInt();
        haoze.rec[servo_id]=Serial.read();
        //转换成signed char(范围-128~127),esp8266默认char是unsigned char，范围0~255
        haoze.rec[servo_id]=Serial.parseInt();
        EEPROM.write(Servo_Addr+servo_id,haoze.rec[servo_id]);
        EEPROM.commit();
      }
      haoze.printcaliData();//打印关节标定数据
      for(int i=0;i<18;i++)
      Servo_p[i] = float(rec[i]);
      haoze.framewrite(haoze.Servo0);
      delay(100);
    }    
  }
    if(cmd == 'a')          //前进
    {
      if(gait==0)//如果是三角步态
      {
        if(gait_zhen>39)
        {
          gait_zhen = 0;
        }
        if(body == 0)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda[gait_zhen][i];
        }
        else if(body == 1)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda2[gait_zhen][i];
        }
        else if(body == 2)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda3[gait_zhen][i];
        }
        
        haoze.framewrite(Servo_p);
        delay(haoze.speed);
        ESP.wdtFeed();                    //喂狗防止复位
        gait_zhen++;
        }
      else if(gait==1)//如果是波动步态
      {
        if(gait_zhen>59)
        gait_zhen = 0;
      
        for(int i=0;i<18;i++)
        Servo_p[i] = forwardaF2[gait_zhen][i];
        
        haoze.framewrite(Servo_p);
        delay(haoze.speed);
        ESP.wdtFeed();                    //喂狗防止复位
        gait_zhen++;
      }
      
    }
    else if(cmd == 'b')//后退
    {
      if(gait==0)//如果是三角步态
      {
        if(gait_zhen<0)
        gait_zhen = 39;
        
        if(body == 0)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda[gait_zhen][i];
        }
        else if(body == 1)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda2[gait_zhen][i];
        }
        else if(body == 2)
        {
          for(int i=0;i<18;i++)
          Servo_p[i] = forwarda3[gait_zhen][i];
        }
        
        haoze.framewrite(Servo_p);
        ESP.wdtFeed();                    //喂狗防止复位
        delay(haoze.speed);
//        delay(deltr);
        gait_zhen--;
      }
      else if(gait==1)//如果是波动步态
      {
        if(gait_zhen<0)
        gait_zhen = 59;

        for(int i=0;i<18;i++)
        Servo_p[i] = forwardaF2[gait_zhen][i];
        
        haoze.framewrite(Servo_p);
        ESP.wdtFeed();                    //喂狗防止复位
        delay(deltr);
        gait_zhen--;
      }
    }
    else if(cmd == 'c')
    {
      if(gait==0)//如果是三角步态
      {
        if(body == 0)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[20+j][i];

            Servo_p[15] = forwarda[20+j][15];
            Servo_p[16] = forwarda[20+j][16];
            Servo_p[17] = forwarda[20+j][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed();                    //喂狗防止复位
              delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[j-20][i];

            Servo_p[15] = forwarda[j-20][15];
            Servo_p[16] = forwarda[j-20][16];
            Servo_p[17] = forwarda[j-20][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed();                    //喂狗防止复位
              delayMicroseconds(del);
          }    
        }
        else if(body == 1)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[20+j][i];

            Servo_p[15] = forwarda2[20+j][15];
            Servo_p[16] = forwarda2[20+j][16];
            Servo_p[17] = forwarda2[20+j][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed();                    //喂狗防止复位
              delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[j-20][i];

            Servo_p[15] = forwarda2[j-20][15];
            Servo_p[16] = forwarda2[j-20][16];
            Servo_p[17] = forwarda2[j-20][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed();                    //喂狗防止复位
              delayMicroseconds(del);
          }    
        }
        else if(body == 2)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[20+j][i];

            Servo_p[15] = forwarda3[20+j][15];
            Servo_p[16] = forwarda3[20+j][16];
            Servo_p[17] = forwarda3[20+j][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed();                    //喂狗防止复位
              delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[j-20][i];

            Servo_p[15] = forwarda3[j-20][15];
            Servo_p[16] = forwarda3[j-20][16];
            Servo_p[17] = forwarda3[j-20][17];
            
            //左边三条腿       
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[39-j][i];
                
            haoze.framewrite(Servo_p);
            delay(haoze.speed);
              ESP.wdtFeed();                    //喂狗防止复位
              delayMicroseconds(del);
          }    
        }
      }
      else if(gait==1)//如果是波动步态
      {
          for(int j=0;j<60;j++)
          {
            //右边腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwardaF2[j][i];

            Servo_p[15] = forwardaF2[j][15];
            Servo_p[16] = forwardaF2[j][16];
            Servo_p[17] = forwardaF2[j][17];

            //左边腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwardaF2[59-j][i];

            haoze.framewrite(Servo_p);
              delay(haoze.speed);
              ESP.wdtFeed();                    //喂狗防止复位
              delay(0);
          }        
      }
    }
    else if(cmd == 'd')
    {
      if(gait==0)//如果是三角步态
      {
        if(body == 0)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[39-j][i];

            Servo_p[15] = forwarda[39-j][15];
            Servo_p[16] = forwarda[39-j][16];
            Servo_p[17] = forwarda[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[20+j][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();                    //喂狗防止复位
            delayMicroseconds(del);
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda[39-j][i];

            Servo_p[15] = forwarda[39-j][15];
            Servo_p[16] = forwarda[39-j][16];
            Servo_p[17] = forwarda[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda[j-20][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();                    //喂狗防止复位
          }
        }
        else if(body == 1)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[39-j][i];

            Servo_p[15] = forwarda2[39-j][15];
            Servo_p[16] = forwarda2[39-j][16];
            Servo_p[17] = forwarda2[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[20+j][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();                    //喂狗防止复位
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda2[39-j][i];

            Servo_p[15] = forwarda2[39-j][15];
            Servo_p[16] = forwarda2[39-j][16];
            Servo_p[17] = forwarda2[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda2[j-20][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();                    //喂狗防止复位
          }
        }
        else if(body == 2)
        {
          for(int j=0;j<20;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[39-j][i];

            Servo_p[15] = forwarda3[39-j][15];
            Servo_p[16] = forwarda3[39-j][16];
            Servo_p[17] = forwarda3[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[20+j][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();                    //喂狗防止复位
          } 
          for(int j=20;j<40;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwarda3[39-j][i];

            Servo_p[15] = forwarda3[39-j][15];
            Servo_p[16] = forwarda3[39-j][16];
            Servo_p[17] = forwarda3[39-j][17];

            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwarda3[j-20][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();                    //喂狗防止复位
          }
        }
      }
      else if(gait==1)//如果是波动步态
      {
          for(int j=0;j<60;j++)
          {
            //右边三条腿
            for(int i=0;i<6;i++)
            Servo_p[i] = forwardaF2[59-j][i];

            Servo_p[15] = forwardaF2[59-j][15];
            Servo_p[16] = forwardaF2[59-j][16];
            Servo_p[17] = forwardaF2[59-j][17];
            
            //左边三条腿
            for(int i=6;i<15;i++)
            Servo_p[i] = forwardaF2[j][i];

            haoze.framewrite(Servo_p);
            delay(haoze.speed);
            ESP.wdtFeed();                    //喂狗防止复位
            delay(0);
          }        
      }      
    }
    else if(cmd == 'f')//向左横移
    {
      for(int j=0;j<60;j++)
      {
          haoze.framewrite(forwardaFH[j]);
          delay(haoze.speed);
          ESP.wdtFeed();                    //喂狗防止复位
      }
    }
    else if(cmd == 'g')//向右横移
    {
      for(int j=0;j<60;j++)
      {
          haoze.framewrite(forwardaFH[59-j]);
          delay(haoze.speed);
          ESP.wdtFeed();                    //喂狗防止复位
      }      
    }
    else if(cmd == 'h')//切换步态
    {
      gait=!gait;
      cmd = 'e';
    }
    else if(cmd == 'i')//切换身高
    {
      body++;
      body=body%3;
      cmd = 'e';
    }
    else if(cmd == 'j')//原地蹲起
    {
          for(int j=0;j<40;j++)
          {
              haoze.framewrite(dunqia[j]);
              delay(deltr*10);
              ESP.wdtFeed();                    //喂狗防止复位
          }
    }
    else if(cmd == 'k')//跳跳
    {
          for(int j=0;j<40;j++)
          {
              haoze.framewrite(benga[j]);
              delay(deltr*30);
              ESP.wdtFeed();                    //喂狗防止复位
          }
    }    
    else
    {
      delay(100);
      ESP.wdtFeed();                    //喂狗防止复位
    }
}
